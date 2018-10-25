#! /usr/bin/env python2.7
# -*- encoding: UTF-8 -*-

import argparse
import contextlib
import glob
import os
import subprocess
import wave
from argparse import RawTextHelpFormatter

import cv2
import numpy as np
import rosbag
import scipy.io.wavfile as wavfile
from cv_bridge import CvBridge

bridge = CvBridge()

__author__ = 'Atef Ben Youssef'

opt_display_images = False;
parser = argparse.ArgumentParser(
    description="Export audio-video from rosbag files and synchronize them using timestamps in a single video\n"
                "Note: avconv and ffmpeg should be installed in your system.",
    formatter_class=RawTextHelpFormatter)
parser.add_argument('-i', '--input', help='Input bagfile', required=True)
parser.add_argument('-o', '--output', help='Output directery of audiovisual files')
parser.add_argument('-f', '--fps', help='frequency frame per second', default=30)
args = parser.parse_args()


def print_log(bagfile, output, path, filename):
    """
        print the timestamps of each image in a file
    """
    bag = rosbag.Bag(bagfile)
    o = open(output + '/' + filename + '_filetime.csv', 'w')
    o.write('topic,secs,nsecs\n')
    for topic, msg, t in bag.read_messages():
        if topic == '/naoqi_driver_node/camera/front/image_raw':
            o.write("front,%d,%d,%f\n" % (t.secs, t.nsecs, msg.header.stamp.to_sec()))
        elif topic == '/naoqi_driver_node/camera/bottom/image_raw':
            o.write("bottom,%d,%d,%f\n" % (t.secs, t.nsecs, msg.header.stamp.to_sec()))
        elif topic == '/naoqi_driver_node/audio':
            o.write("audio,%d,%d,%f\n" % (t.secs, t.nsecs, msg.header.stamp.to_sec()))
    o.close()
    bag.close()
    print (output + '/' + filename + '_time.csv')


def get_sound(bagfile, output, path, filename, bag_topic):
    """
        compute the audio from bagfile and save it in a wave file
    """
    # if not os.path.exists(output + '/' + filename + '.wav'):
    global dur_wav
    bag = rosbag.Bag(bagfile)
    # audio = []
    audiosamples = []
    print ("Extration...")
    time_deb = 0
    tdur = 0
    wdur = 0
    nchan = 4
    if 'wavsamples' in locals():
        del wavsamples
    for topic, msg, t in bag.read_messages(topics=bag_topic):
        if topic == '/naoqi_driver_node/audio':
            if time_deb == 0:
                time_deb = msg.header.stamp.to_sec()
                tprec = t
            time_fin = msg.header.stamp.to_sec()
            nchan = len(msg.channelMap)
            sr = msg.frequency
            # audio.extend(msg.data)
            tsample = ((np.asarray(msg.data)).astype('int16')).reshape((len(msg.data) / nchan, nchan))
            tdur = tdur + t.to_sec() - tprec.to_sec()
            wdur = wdur + float(len(tsample)) / float(sr)
            # print ("Timestamp = " + str(t.to_sec()-tprec.to_sec()) + " & wav trame dur = " + str(float(len(tsample))/float(sr)))
            # print ("Timestamp Dur = " + str(tdur) + " & wav trame dur = " + str(wdur))
            audiosamples.extend(msg.data)
           # if 'wavsamples' in locals():
           #     wavsamples = np.concatenate((wavsamples,((np.asarray(msg.data)).astype('int16')).reshape(len(msg.data)/nchan,nchan)))
           # else:
           #     wavsamples = ((np.asarray(msg.data)).astype('int16')).reshape(len(msg.data)/nchan,nchan)
            if not opt_display_images:
                while (tdur >= wdur):
                    # audiosamples.extend(msg.data)
                    audiosamples.extend([0] * len(msg.data))
                    # wavsamples = np.concatenate((wavsamples,((np.asarray(msg.data)).astype('int16')).reshape(len(msg.data)/nchan,nchan)))
                    # print ("+ Timestamp Dur = " + str(tdur) + " & wav trame dur = " + str(wdur))
                    wdur = wdur + float(len(tsample)) / float(sr)
            tprec = t
    # nsamples = len(audio)/nchan
    naudiosamples = len(audiosamples) / nchan
    #        naudiosamples = np.shape(wavsamples)[0]
    print ('NbChannel = ' + str(nchan))
    print ('Frequency = ' + str(sr))
    # print ('#samples = ' + str(nsamples))
    print ('#Timestamps samples = ' + str(naudiosamples))

    print ('\nOutput wave file :')
    # convert the samples from float to int16
    # samples = ((np.asarray(audio)).astype('int16')).reshape((nsamples,nchan))
    wavsamples = ((np.asarray(audiosamples)).astype('int16')).reshape(naudiosamples, nchan)
    # plt.plot(wavsamples[0:16384*12])
    print ("Bag dur = " + str(float(time_fin - time_deb)))
    # print ("wav dur = " + str(float(len(samples))/float(sr)))
    print ("wav trame dur = " + str(float(len(wavsamples)) / float(sr)))
    dur_wav = float(len(wavsamples)) / float(sr)
    print (output + '/' + filename + '.wav')
    if abs(time_fin - time_deb) - (len(wavsamples) / sr) > 1:
        print ('WARNING! Duration difference between bag and wav = ' + str(
            abs(time_fin - time_deb) - (len(wavsamples) / sr)))
        dur = abs(time_fin - time_deb) - (len(wavsamples) / sr)

    wavfile.write(output + '/' + filename + '.wav', sr, wavsamples)
    # wavfile.write(output + '/' + filename + '_nogap.wav',sr,samples)
    # downsample and combine channels
    #        cmd = ['sox',output + '/' + filename + '.wav','-c1','-r','16000',output + '/' + filename + '_16k_mono.wav']
    cmd = ['sox', output + '/' + filename + '.wav', '-c1', output + '/' + filename + '_mono.wav']
    p = subprocess.Popen(cmd)
    p.wait()
    print (output + '/' + filename + '_mono.wav')
    bag.close()


def extract_images(bagfile, output, path, filename, opt_fps, bag_topic):
    """
        compute the images from the different cameras and save them in an avi video file
    """
    # if not os.path.exists(output + '/' + filename + '_bottom.avi'):
    global img_fps
    t_first = {};
    t_file = {};
    t_video = {}
    cv_image = []
    np_arr = []
    nbframe = 0;
    if (opt_fps <= 0):
        opt_fps = 1
    print ("using ", opt_fps, " FPS")

    if opt_display_images:
        try:
            os.stat(output + '/' + filename + '_front')
        except:
            os.mkdir(output + '/' + filename + '_front')
        try:
            os.stat(output + '/' + filename + '_bottom')
        except:
            os.mkdir(output + '/' + filename + '_bottom')

    p_avconv = {}
    if os.path.exists(output + '/' + filename + '_extract_front.avi'):
        os.remove(output + '/' + filename + '_extract_front.avi')
    if os.path.exists(output + '/' + filename + '_extract_bottom.avi'):
        os.remove(output + '/' + filename + '_extract_bottom.avi')
    bag = rosbag.Bag(bagfile)
    time_deb = 0
    nb_imgf = 0
    nb_imgb = 0
    tdur = 0
    if opt_display_images:
        o = open(output + '/' + filename + '_imgtimestamp.csv', 'w')
        o.write('image,timestamp\n')
    # for topic, msg, t in bag.read_messages(connection_filter=filter_image_msgs):
    for topic, msg, t in bag.read_messages(topics=bag_topic):
        if topic == '/naoqi_driver_node/camera/front/image_raw':
            if time_deb == 0:
                time_deb = msg.header.stamp.to_sec()
                # tprec = t
            time_fin = msg.header.stamp.to_sec()
            pix_fmt = ""
            if msg.encoding.find("mono8") != -1:
                pix_fmt = "gray"
                if opt_display_images:
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    imgstr = "%04d" % nb_imgf
                    image_name = output + '/' + filename + '_front/image_' + imgstr + '.jpg'
                    o.write("%s,%f\n" % (image_name, msg.header.stamp.to_sec()))
                    cv2.imwrite(image_name, cv_image)
                    nb_imgf = nb_imgf + 1
            elif msg.encoding.find("bgr8") != -1:
                pix_fmt = "bgr24"
                if opt_display_images:
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    imgstr = "%04d" % nb_imgf
                    image_name = output + '/' + filename + '_front/image_' + imgstr + '.jpg'
                    o.write("%s,%f\n" % (image_name, msg.header.stamp.to_sec()))
                    cv2.imwrite(image_name, cv_image)
                    nb_imgf = nb_imgf + 1
            elif msg.encoding.find("rgb8") != -1:
                pix_fmt = "rgb24"
                if opt_display_images:
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    imgstr = "%04d" % nb_imgf
                    image_name = output + '/' + filename + '_front/image_' + imgstr + '.jpg'
                    o.write("%s,%f\n" % (image_name, msg.header.stamp.to_sec()))
                    cv2.imwrite(image_name, cv_image)
                    nb_imgf = nb_imgf + 1
            else:
                print ('unsuportet encoding:', msg.encoding)
                exit(1)

            if not opt_display_images:
                print (topic, 'at', str(t), 'msg=', str(msg.height))
                pix_fmt = "rgb24"
                if len(msg.data) > 0:
                    if not topic in t_first:
                        t_first[topic] = t;
                        t_video[topic] = 0;
                        t_file[topic] = 0
                    t_file[topic] = (t - t_first[topic]).to_sec()
                    while t_video[topic] < t_file[topic]:
                        if not topic in p_avconv:
                            size = str(msg.width) + "x" + str(msg.height)
                            p_avconv[topic] = subprocess.Popen(
                                ['avconv', '-r', str(opt_fps), '-an', '-f', 'rawvideo', '-s', size, '-pix_fmt', pix_fmt,
                                 '-i', '-', output + '/' + filename + '_extract_front.avi'], stdin=subprocess.PIPE)
                        p_avconv[topic].stdin.write(msg.data)
                        t_video[topic] += 1.0 / opt_fps
                    print (output + '/' + filename + '_extract_front.avi ' + str(t_video[topic]) + ' ' + str(
                        t_file[topic]))

        if topic == '/naoqi_driver_node/camera/bottom/image_raw':
            if time_deb == 0:
                time_deb = msg.header.stamp.to_sec()
                # tprec = t
            pix_fmt = ""
            # tdur = tdur + t.to_sec()-tprec.to_sec()
            # tprec = t
            if msg.encoding.find("mono8") != -1:
                pix_fmt = "gray"
                if opt_display_images:
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    imgstr = "%04d" % nb_imgb
                    image_name = output + '/' + filename + '_bottom/image_' + imgstr + '.jpg'
                    cv2.imwrite(image_name, cv_image)
                    nb_imgb = nb_imgb + 1
            elif msg.encoding.find("bgr8") != -1:
                pix_fmt = "bgr24"
                if opt_display_images:
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    imgstr = "%04d" % nb_imgb
                    image_name = output + '/' + filename + '_bottom/image_' + imgstr + '.jpg'
                    cv2.imwrite(image_name, cv_image)
                    nb_imgb = nb_imgb + 1
            elif msg.encoding.find("rgb8") != -1:
                pix_fmt = "rgb24"
                if opt_display_images:
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    imgstr = "%04d" % nb_imgb
                    image_name = output + '/' + filename + '_bottom/image_' + imgstr + '.jpg'
                    cv2.imwrite(image_name, cv_image)
                    nb_imgb = nb_imgb + 1
            else:
                print ('unsuportet encoding:', msg.encoding)
                exit(1)

            if not opt_display_images:
                print (topic, 'at', str(t), 'msg=', str(msg.height))
                pix_fmt = "rgb24"
                if len(msg.data) > 0:
                    if not topic in t_first:
                        t_first[topic] = t;
                        t_video[topic] = 0;
                        t_file[topic] = 0
                    t_file[topic] = (t - t_first[topic]).to_sec()
                    while t_video[topic] < t_file[topic]:
                        if not topic in p_avconv:
                            size = str(msg.width) + "x" + str(msg.height)
                            p_avconv[topic] = subprocess.Popen(
                                ['avconv', '-r', str(opt_fps), '-an', '-f', 'rawvideo', '-s', size, '-pix_fmt', pix_fmt,
                                 '-i', '-', output + '/' + filename + '_extract_bottom.avi'], stdin=subprocess.PIPE)
                        p_avconv[topic].stdin.write(msg.data)
                        t_video[topic] += 1.0 / opt_fps
                    print (output + '/' + filename + '_extract_bottom.avi ' + str(t_video[topic]) + ' ' + str(
                        t_file[topic]))

        # print ("Depth camera files...")
        # if topic == '/naoqi_driver_node/camera/depth/image_raw':
        # pix_fmt=""
        # if msg.encoding.find("mono8")!=-1 :
        # pix_fmt = "gray"
        # #np_arr = np.fromstring(msg.data, np.uint8)
        # if opt_display_images:
        # cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # elif msg.encoding.find("bgr8")!=-1 :
        # pix_fmt = "bgr24"
        # #np_arr = np.fromstring(msg.data, np.uint8)
        # if opt_display_images:
        # cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # elif msg.encoding.find("rgb8")!=-1 :
        # pix_fmt = "rgb24"
        # #np_arr = np.fromstring(msg.data, np.uint8)
        # if opt_display_images:
        # cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # else:
        # print 'unsuportet encoding:', msg.encoding
        # exit(1)
        # print topic, 'at', str(t),'msg=', str(msg.height)
        # pix_fmt = "rgb24"
        # if len(msg.data)>0:
        # if not topic in t_first :
        # t_first[topic] = t;
        # t_video[topic] = 0;
        # t_file[topic] = 0
        # t_file[topic] = (t-t_first[topic]).to_sec()
        # while t_video[topic]<t_file[topic]:
        # if not topic in p_avconv:
        # size = str(msg.width)+"x"+str(msg.height)
        # p_avconv[topic] = subprocess.Popen(['avconv','-r',str(opt_fps),'-an','-f','rawvideo','-s',size,'-pix_fmt', pix_fmt,'-i','-',output + '/' + filename + '_extract_depth.avi'],stdin=subprocess.PIPE)
        # p_avconv[topic].stdin.write(msg.data)
        # t_video[topic] += 1.0/opt_fps
        # print (output + '/' + filename + '_extract_depth.avi ' + str(t_video[topic]) + ' ' + str(t_file[topic]))
    if opt_display_images:
        o.close()

    bag.close()


def synchronize(bagfile, output, path, filename, opt_fps, offset=0):
    """
        create a synchronous audio+front-and-bottom-video
    """
    # put it all together with ffmpeg
    # cmd = ['ffmpeg','-i',output + '/' + filename + '_extract_front.avi','-i',output + '/' + filename + '.wav','-vcodec','copy','-acodec','copy','-y',output + '/' + filename + '_front_audiovideo.avi']
    # p = subprocess.Popen(cmd)
    # p.wait()

    if opt_display_images:
        with contextlib.closing(wave.open(output + '/' + filename + '.wav', 'r')) as f:
            frames = f.getnframes()
            rate = f.getframerate()
            duration = frames / float(rate)
        dur_wav = duration
        nb_imgf = len(glob.glob(output + '/' + filename + '_front/image_*.jpg'))
        vid_fps = float(nb_imgf) / float(dur_wav)
        # vid_fps = float(dur_wav) / float(nb_imgf)
        cmd = ['ffmpeg', '-framerate', str(vid_fps), '-i', output + '/' + filename + '_front/image_%04d.jpg', '-vf',
               'fps=30', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-r', '30', '-shortest', '-y', '-t', str(dur_wav),
               output + '/' + filename + '_extract_front.avi']
        print (subprocess.list2cmdline(cmd))
        p = subprocess.Popen(cmd)
        p.wait()
        nb_imgb = len(glob.glob(output + '/' + filename + '_bottom/image_*.jpg'))
        vid_fps = float(nb_imgb) / float(dur_wav)
        #            vid_fps = float(dur_wav) / float(nb_imgb)
        cmd = ['ffmpeg', '-framerate', str(vid_fps), '-i', output + '/' + filename + '_bottom/image_%04d.jpg', '-vf',
               'fps=30', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-r', '30', '-shortest', '-y', '-t', str(dur_wav),
               output + '/' + filename + '_extract_bottom.avi']
        print (subprocess.list2cmdline(cmd))
        p = subprocess.Popen(cmd)
        p.wait()

    cmd = ['ffmpeg', '-i', filename + '_extract_front.avi', '-i', filename + '.wav', '-vf',
           'pad=iw:2*ih [top]; movie=' + filename + '_extract_bottom.avi [bottom]; [top][bottom] overlay=0:main_h/2',
           '-acodec', 'copy', '-y', filename + '_frontbottom_audiovideo.avi']
    print (subprocess.list2cmdline(cmd))
    p = subprocess.Popen(cmd, cwd=output)
    p.wait()


if __name__ == "__main__":
    baginput = args.input
    outputdir = args.output
    fps = args.fps
    # baginput = '/media/atef/UEHRI/user181_2017-05-05.bag'
    # baginput = '/media/atef/UEHRI/user53_2017-02-02.bag'
    # outputdir = '/media/atef/UEHRI/Videos_Interaction/'
    # fps = 30

    opt_fps = float(fps)
    pathabs = os.path.abspath(baginput)
    path = os.path.dirname(pathabs)
    if outputdir is not None:
        outputdir = outputdir
    else:
        outputdir = path
    try:
        os.stat(outputdir)
    except:
        os.mkdir(outputdir)

    print ("Input bag file: %s" % baginput)
    print ("Output directory: %s" % outputdir)
    # print ("Input fps: %s" % opt_fps)

    filename, ext = os.path.splitext(baginput)
    basenamefile = os.path.basename(filename)
    print ("Basename: %s" % basenamefile)
    # print ("Time file...")
    # print_log(fbag,outputdir,path,basenamefile)
    print ("Audio file...")
    get_sound(baginput, outputdir, path, basenamefile, '/naoqi_driver_node/audio')
    print ("Video files...")
    extract_images(baginput, outputdir, path, basenamefile, opt_fps,
                   ['/naoqi_driver_node/camera/front/image_raw', '/naoqi_driver_node/camera/bottom/image_raw'])
    print ("Synchronizing audio and videos files...")
    synchronize(baginput, outputdir, path, basenamefile, opt_fps)
    print ("\nDone: " + outputdir + "/" + basenamefile + "*")
