import rospy, signal, sys, time
import numpy as np
from functools import partial

from run_visualization_engine import Timer#VisualizerClass, Timer
from visualizerclass import VisualizerClass
from std_msgs.msg import Header, ColorRGBA, Int32
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Vector3

def signal_handler(timer, signal, frame):

    print("Exit the attention engine.\n")
    timer.finish()
    # sys.exit(0) # this is not necessary

if __name__ == '__main__':

    # Initialize node
    rospy.init_node('visualization_engine', anonymous=True)
    print ("Start visualzation_engine.")
    tmr_plot = Timer(_name='Plot',_HZ=500,_MAX_SEC=np.inf,_VERBOSE=True)
    
    # Set terminator
    signal.signal(signal.SIGINT, partial(signal_handler,tmr_plot))

    # Visualizer
    V = VisualizerClass(name='simple viz',HZ=500)
    
    # Start the loop 

    ps_list = np.load('ps_list.npy')
    target_ps_list = np.load('ps_list2.npy')

    n_body = ps_list.shape[1]
    max_tick = ps_list.shape[0]

    tmr_plot.start()
    while tmr_plot.is_notfinished(): # loop 
        if tmr_plot.do_run(): # plot (20HZ)

            tick = tmr_plot.tick
            # print ("Plot [%.3f]sec."%(tmr_plot.sec_elps))

            # Reset 
            V.reset_markers()            
            # Append marker
            for body_idx in range(n_body):

                x,y,z,r = ps_list[int(tick%max_tick),body_idx,0], ps_list[int(tick%max_tick),body_idx,1], ps_list[int(tick%max_tick),body_idx,0],0.1#np.sin(tick/5),np.cos(tick/5),0,0.5
                V.append_marker(x=x,y=y,z=z,frame_id='map',
                    color=ColorRGBA(1.0,0.0,float(body_idx)/n_body,1.),marker_type=Marker.SPHERE,scale=Vector3(0.1,0.1,0.1),)
                x,y,z,r = target_ps_list[int(tick%max_tick),body_idx,0], ps_list[int(tick%max_tick),body_idx,1], ps_list[int(tick%max_tick),body_idx,0],0.1#np.sin(tick/5),np.cos(tick/5),0,0.5
                V.append_marker(x=x,y=y,z=z,frame_id='map',
                    color=ColorRGBA(0.0,1.0,float(body_idx)/n_body,1.),marker_type=Marker.SPHERE,scale=Vector3(0.1,0.1,0.1),)

            # x,y,z,r = 0,np.sin(tick/5),np.cos(tick/5),0.5
            # V.append_marker(x=x,y=y,z=z,r=r,frame_id='map',
            #     color=ColorRGBA(0.0,0.0,1.0,0.5),marker_type=Marker.CUBE)
            # x,y,z,r = np.sin(tick/5),0,np.cos(tick/5),0.5
            # V.append_marker(x=x,y=y,z=z,r=r,frame_id='map',
            #     color=ColorRGBA(0.0,1.0,0.0,0.5),marker_type=Marker.CYLINDER)

            # Publish
            V.publish_markers()
            tmr_plot.end()

        rospy.sleep(1e-8)

    # Exit handler here
    V.delete_markers()
    