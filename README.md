# UE-HRI
UE-HRI is a dataset collected for the study of user engagement in spontaneous Human-Robot Interactions (HRI).
http://www.tsi.telecom-paristech.fr/aao/en/2017/05/18/ue-hri-dataset/

## Scripts to extract feature from bag files.
```
usage: export_audiovideo_timestamps.py [-h] -i INPUT [-o OUTPUT] [-f FPS]

Export audio-video from rosbag files and synchronize them using timestamps in a single video
Note: avconv and ffmpeg should be installed in your system.

optional arguments:
  -h, --help            show this help message and exit
  -i INPUT, --input INPUT
                        Input bagfile
  -o OUTPUT, --output OUTPUT
                        Output directery of audiovisual files
  -f FPS, --fps FPS     frequency frame per second

```
