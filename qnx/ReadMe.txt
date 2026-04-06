QNX Steps:
1. Import Project (VisionProd) and build project
2. move frame_feeder_qnx, parking_manager_qnx, empty_ref_1100x720.raw, and roi_map_output.json into virtual machine (vm1_demo)
3. in virtual machine you have to setup the TCP receiver using the following (it might be different for you):
	a. ifconfig vtnet0 down
	b. ifconfig vtnet0 192.168.56.20 netmask 255.255.255.0 up
	c. ifconfig vtnet0
(you may want to ping test in powershell, also note that the ip address when you shut off the virtual machine so we have to redo the set-up for vtnet0)
4. run the following commands in virtual machine:
	a. ./parking_manager_qnx > parking_results.txt 2>&1 & (runs the parking manager in the background and outputs the results into a text file
	b. ./frame_feeder_qnx 5000 roi_map_output.json --ref empty_ref_1100x720.raw 1100 720 (the port is 5000 make sure it is not blocked in windows defender)
	frame_feeder will be waiting for the camera_stream.py to connect to it

Windows Steps:
1. In a file put camera_stream.py and carPark.mp4
2. run the following command in powershell in the same directory: python ./camera_stream.py carPark.mp4 --host 192.168.56.20 --port 5000
It will output the following:
[CS] source  : carPark.mp4
[CS] native  : 1100x720 @ 24.00 fps
[CS] frames  : 679
[CS] target  : 192.168.56.20:5000
[CS] output  : 1100x720 @ 24.00 fps
[CS] loop    : false
[CS] TCP connected
[CS] source exhausted – 679 frames sent
[CS] done

You will also see frames being fed into vision_producer in virtual machine