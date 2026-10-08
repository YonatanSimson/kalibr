import sm

import numpy as np
import sys
import multiprocessing
try:
   import queue
except ImportError:
   import Queue as queue # python 2.x
import time
import copy
import cv2
import json
import os
import re
import aslam_cv as acv

def multicoreExtractionWrapper(detector, taskq, resultq, clearImages, noTransformation):    
    while 1:
        try:
            task = taskq.get_nowait()
        except queue.Empty:
            return
        idx = task[0]
        stamp = task[1]
        image = task[2]
        
        if noTransformation:
            success, obs = detector.findTargetNoTransformation(stamp, np.array(image))
        else:
            success, obs = detector.findTarget(stamp, np.array(image))
            
        if clearImages:
            obs.clearImage()
        if success:
            resultq.put( (obs, idx) )

def importCornersFromJson(path, dataset, detector, clearImages=True, noTransformation=False):
    """Observations from externally detected AprilGrid corners instead of kalibr's detector.

    `path` is a basalt_dump_corners JSON: [{frame_id: timestamp ns, cam_id, corner_ids, corners}],
    corner_id = 4 * tag_id + k with k = bottom-left, bottom-right, top-right, top-left -- the same
    tag numbering and corner order as kalibr's AprilGrid, so corner (t, k) is grid point
    base(t) + [0, 1, cols + 1, cols][k]. The lens is taken from the bag topic (/camN/...). Frames are
    still read from the bag (image size and timestamp), matched to the JSON within 1 ms. Basalt's
    detector keeps far more of the curved / compressed tags at the rim of a > 180 deg fisheye.

    Several files ("a.json:b.json", e.g. two recordings of the same camera and mode, pooled for
    intrinsics): observations are built from the JSON frames alone -- the bag only provides the image
    size -- so no combined bag is needed. KALIBR_CORNERS_EVERY=N keeps every Nth frame per file (the
    bag's --bag-freq does not apply in this mode). Images are not kept (kalibr's corner plots skip
    them).
    """
    paths = path.split(":")
    if len(paths) > 1:
        return importCornersFromJsonOnly(paths, dataset, detector, noTransformation)
    m = re.search(r"cam(\d+)", dataset.topic)
    if m is None:
        raise RuntimeError("KALIBR_CORNERS_JSON: cannot tell the camera id from topic {0}".format(dataset.topic))
    cam_id = int(m.group(1))
    with open(path) as f:
        frames = {int(round(fr["frame_id"] * 1e-6)): fr for fr in json.load(f) if fr["cam_id"] == cam_id}
    target = detector.target()
    cols = target.cols()          # corner-grid columns = 2 * tag columns
    tag_cols = cols // 2
    min_corners = 4 * 4           # as kalibr's AprilGrid default (4 tags)
    print("Importing corners for {0} (cam_id {1}) from {2}: {3} frames".format(dataset.topic, cam_id, path, len(frames)))

    observations = []
    for timestamp, image in dataset.readDataset():
        fr = frames.get(int(round(timestamp.toSec() * 1e3)))
        if fr is None or len(fr["corner_ids"]) < min_corners:
            continue
        obs = acv.GridCalibrationTargetObservation(target)
        obs.setImage(np.array(image))
        obs.setTime(timestamp)
        for cid, uv in zip(fr["corner_ids"], fr["corners"]):
            t, k = cid // 4, cid % 4
            base = (t // tag_cols) * cols * 2 + (t % tag_cols) * 2
            obs.updateImagePoint(base + (0, 1, cols + 1, cols)[k], np.array(uv, dtype=float))
        if not noTransformation:
            success, T_t_c = detector.geometry().estimateTransformation(obs)
            if not success:
                continue
            obs.set_T_t_c(T_t_c)
        if clearImages:
            obs.clearImage()
        observations.append(obs)
    print("  imported {0} observations".format(len(observations)))
    return observations

def _corner_frames(path, cam_id):
    with open(path) as f:
        return [fr for fr in json.load(f) if fr["cam_id"] == cam_id]

def _observation_from_frame(fr, target, blank, timestamp, detector, noTransformation):
    cols = target.cols()          # corner-grid columns = 2 * tag columns
    tag_cols = cols // 2
    obs = acv.GridCalibrationTargetObservation(target)
    obs.setImage(blank)           # sets the image size used by the intrinsics initialisation
    obs.setTime(timestamp)
    for cid, uv in zip(fr["corner_ids"], fr["corners"]):
        t, k = cid // 4, cid % 4
        base = (t // tag_cols) * cols * 2 + (t % tag_cols) * 2
        obs.updateImagePoint(base + (0, 1, cols + 1, cols)[k], np.array(uv, dtype=float))
    if not noTransformation:
        success, T_t_c = detector.geometry().estimateTransformation(obs)
        if not success:
            return None
        obs.set_T_t_c(T_t_c)
    obs.clearImage()
    return obs

def importCornersFromJsonOnly(paths, dataset, detector, noTransformation=False):
    """Several corner files pooled; observations from the JSON frames only (see importCornersFromJson)."""
    m = re.search(r"cam(\d+)", dataset.topic)
    if m is None:
        raise RuntimeError("KALIBR_CORNERS_JSON: cannot tell the camera id from topic {0}".format(dataset.topic))
    cam_id = int(m.group(1))
    every = max(1, int(os.environ.get("KALIBR_CORNERS_EVERY", "1")))
    _, image = next(iter(dataset.readDataset()))
    blank = np.zeros(np.array(image).shape, dtype=np.uint8)
    target = detector.target()
    min_corners = 4 * 4
    observations = []
    for path in paths:
        frames = sorted(_corner_frames(path, cam_id), key=lambda fr: fr["frame_id"])[::every]
        n0 = len(observations)
        for fr in frames:
            if len(fr["corner_ids"]) < min_corners:
                continue
            ns = int(fr["frame_id"])
            obs = _observation_from_frame(fr, target, blank, acv.Time(ns // 1000000000, ns % 1000000000),
                                          detector, noTransformation)
            if obs is not None:
                observations.append(obs)
        print("Imported {0} observations for {1} (cam_id {2}) from {3} (every {4})".format(
            len(observations) - n0, dataset.topic, cam_id, path, every))
    observations.sort(key=lambda o: o.time().toSec())
    return observations

def extractCornersFromDataset(dataset, detector, multithreading=False, numProcesses=None, clearImages=True, noTransformation=False):
    # KALIBR_CORNERS_JSON: use externally detected corners (see importCornersFromJson).
    corners_json = os.environ.get("KALIBR_CORNERS_JSON")
    if corners_json:
        return importCornersFromJson(corners_json, dataset, detector, clearImages, noTransformation)
    print("Extracting calibration target corners")    
    targetObservations = []
    numImages = dataset.numImages()
    
    # prepare progess bar
    iProgress = sm.Progress2(numImages)
    iProgress.sample()
            
    if multithreading:   
        if not numProcesses:
            numProcesses = max(1,multiprocessing.cpu_count()-1)
        try:      
            manager = multiprocessing.Manager()
            resultq = manager.Queue()
            manager2 = multiprocessing.Manager()
            taskq = manager2.Queue()
            
            for idx, (timestamp, image) in enumerate(dataset.readDataset()):
                taskq.put( (idx, timestamp, image) )
                
            plist=list()
            for pidx in range(0, numProcesses):
                detector_copy = copy.copy(detector)
                p = multiprocessing.Process(target=multicoreExtractionWrapper, args=(detector_copy, taskq, resultq, clearImages, noTransformation, ))
                p.start()
                plist.append(p)
            
            #wait for results
            last_done=0
            while 1:
                if all([not p.is_alive() for p in plist]):
                    time.sleep(0.1)
                    break
                done = numImages-taskq.qsize()
                sys.stdout.flush()
                if (done-last_done) > 0:
                    iProgress.sample(done-last_done)
                last_done = done
                time.sleep(0.5)
            resultq.put('STOP')
        except Exception as e:
            raise RuntimeError("Exception during multithreaded extraction: {0}".format(e))
        
        #get result sorted by time (=idx)
        if resultq.qsize() > 1:
            targetObservations = [[]]*(resultq.qsize()-1)
            for lidx, data in enumerate(iter(resultq.get, 'STOP')):
                obs=data[0]; time_idx = data[1]
                targetObservations[lidx] = (time_idx, obs)
            targetObservations = list(zip(*sorted(targetObservations, key=lambda tup: tup[0])))[1]
        else:
            targetObservations=[]
    
    #single threaded implementation
    else:
        for timestamp, image in dataset.readDataset():
            if noTransformation:
                success, observation = detector.findTargetNoTransformation(timestamp, np.array(image))
            else:
                success, observation = detector.findTarget(timestamp, np.array(image))
            if clearImages:
                observation.clearImage()
            if success == 1:
                targetObservations.append(observation)
            iProgress.sample()

    if len(targetObservations) == 0:
        print("\r")
        sm.logFatal("No corners could be extracted for camera {0}! Check the calibration target configuration and dataset.".format(dataset.topic))
    else:    
        print("\r  Extracted corners for %d images (of %d images)                              " % (len(targetObservations), numImages))

    #close all opencv windows that might be open
    cv2.destroyAllWindows()
    
    return targetObservations
