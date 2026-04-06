Linux Baseline by Dhirran.P

Overview
This is the baseline of the QNX code written on Linux:

Current Pipeline:
camera_stream.py -> frame_feeder_linux -> vision_producer -> parking_manager_linux

Differences from QNX solution:
- I replaced the QNX's MsgSend/MsgReceive functions with the local Unix socket IPC
- The parking_manager_linux runs on the dirctory /tmp/parking_pm.sock
- frame_feeder_linux connects to said socket (parking_pm.sock)
- The same camera_stream.py from the QNX solution feeds frames over TCP to frame_feeder_linux

Files Used:
- frame_feeder_linux
- parking_manager_linux
- roi_map_output.json
- empty_ref_1100x720.raw
- camera_stream.py
- carPark.mp4

How to Build:
1. Open a terminal in ./linux_baseline directory
2. Build:
   make
Files now created:
- linux_baseline/frame_feeder_linux
- linux_baseline/parking_manager_linux

Python Requirement (camera_stream.py requires OpenCV):
If not installed run:
   python3 -m pip install opencv-python

How to Run:
1. Use 3 terminals.
2. Terminal 1, Run Start Parking Manager:
   ./parking_manager_linux
Expected output:
   [PM-LINUX ...] Linux Parking Manager started
   [PM-LINUX ...] listening on /tmp/parking_pm.sock
3. Terminal 2, Run Frame Feeder:
   ./frame_feeder_linux 55000 roi_map_output.json --ref empty_ref_1100x720.raw 1100 720
Expected startup output:
   [VP ...] ROI map loaded: 69 spots from roi_map_output.json
   [VP ...] reference frame loaded from empty_ref_1100x720.raw (1100x720, 792000 bytes)
   [VP ...] analysis_thread: started
   [FF-LINUX] listening on port 55000
4. Terminal 3, Run Camera Stream:
   python3 camera_stream.py carPark.mp4 --host 127.0.0.1 --port 55000
Expected output:
   [CS] source  : carPark.mp4
   [CS] native  : 1100x720 @ 24.00 fps
   [CS] frames  : 679
   [CS] target  : 127.0.0.1:55000
   [CS] output  : 1100x720 @ 24.00 fps
   [CS] loop    : false
   [CS] TCP connected
   [CS] source exhausted – 679 frames sent
   [CS] done
5. Inspect Results in Terminal 1, parking_manager_linux will output the parking occupancy transitions:
   [PM-LINUX ...] A-01 UNKNOWN -> OCCUPIED ratio=0.665 frame=7 ...
   [PM-LINUX ...] A-23 OCCUPIED -> FREE ratio=0.155 frame=164 ...
   [PM-LINUX ...] A-52 OCCUPIED -> FREE ratio=0.163 frame=194 ...
6. Inspect Results in Terminal 2, when the vid is done frame_feeder_linux will output something like the follwoing:
   [FF-LINUX] done – 679 frames
   [VP ...] analysis_thread: exiting (frames=679 events=72 avg_lat=... ns max_lat=... ns)
7. Shut down All Programs Still Running with ^C