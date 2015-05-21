# FlatBuddy
## Alpha Status! 
HOME AUTOMATION System based on open source software and low cost hardware.

Prerequesites:
Install following Ubuntu packages: sudo apt-get install python-pip, ros-indigo-desktop-full

Install following python packages: (sudo) pip install SpeechRecognition

If you used sudo to install with pip: change permissions of SpeechRecognition python library:
Ubuntu Trusty: sudo chown -R username /usr/local/lib/python2.7/dist-packages/speech_recognition/

The system is currently composed of a master laptop (brain), IP Cam (Tenvis HD), Kaikun Smart Plugs.
Feel free to contribute to the project in adding more supported devices!
For the Tenvis HD camera I've also written a python library, which can be found here.
