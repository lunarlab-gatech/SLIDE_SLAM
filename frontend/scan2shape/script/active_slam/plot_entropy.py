#! /usr/bin/env python3
# title			:
# description	:
# author		:Yuezhan Tao and Xu Liu

import numpy as np
import matplotlib.pyplot as plt
import glob
import matplotlib

bag_rate = 0.8
# end time 
end_time = 340
# entropy data
data_dir = "/home/sam/bags/yuezhan-bags/"
# load txt file, skip the first row (header)
# glob the directory for all txt files
files = glob.glob(data_dir + "*.txt")
robot_pose_entropy_plot_upper_lim = 0.17

font_size = 10

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

for filename_cur in files:
    print("Plotting: ")
    print(filename_cur)
    entropy_data = np.loadtxt(filename_cur, delimiter=",", skiprows=1)
    # data columns should be TIME, ENTROPY_POSE, ENTROPY_LANDMARK, NUM_VALID_POSES, NUM_VALID_LANDMARKS
    # take data only after ENTROPY_POSE > 1e-5, and ENTROPY_LANDMARK > 1e-5, and NUM_VALID_POSES > 5, and NUM_VALID_LANDMARKS > 3
    entropy_data = entropy_data[entropy_data[:, 1] > 1e-5]
    entropy_data = entropy_data[entropy_data[:, 2] > 1e-5]
    entropy_data = entropy_data[entropy_data[:, 3] > 5]
    entropy_data = entropy_data[entropy_data[:, 4] > 3]
    # get the start time
    start_time = entropy_data[0, 0]
    # shift the time by the start time
    entropy_data[:, 0] -= start_time

    # plot entropy pose / num valid poses, and entropy landmark / num valid landmarks over time in two subplots
    fig, ax = plt.subplots(3, 1, sharex=True)
    
    entropy_data[:, 0] *= bag_rate
    ax[0].plot(entropy_data[:, 0], entropy_data[:, 1] / entropy_data[:, 3], color="red")


    # set the y axis label with the same color as the plot
    ax[0].set_ylabel("Avg. Pose \n Uncertainty", color="red")
    ax[0].set_ylim([0, robot_pose_entropy_plot_upper_lim])
    # put x y label on upper left inside the plot
    ax[0].yaxis.set_label_position("left")
    # plot legend
    # ax[0].legend(["Avg. Pose Uncertainty"], loc="upper left")


    # # overlay the number of num valid poses, with a separate scale and label on the right y axis
    ax2 = ax[0].twinx()
    # dotted plot
    ax2.plot(entropy_data[:, 0], entropy_data[:, 3], color="grey", linestyle="dotted")
    # set the y axis label with the same color as the plot
    ax2.set_ylabel("Num. Poses", color="grey")
    
    
    ax2.set_ylim([0, 600])
    ax2.yaxis.set_label_coords(1.06, 0.5)
    ax2.yaxis.label.set_rotation(45)
    # fixed figure size
    fig.set_size_inches(10, 3.2)

    # plot legend with large and boldfont


    # ax2.legend(["Num. Robot Poses"], loc="center left", fontsize=font_size)


    # y scale clip to 150
    # run a filter to smooth the landmark data
    landmark_entropy_over_time = entropy_data[:, 2] #/ entropy_data[:, 4]

    do_filter = False

    # filter_window = 20
    # total_time = max(entropy_data[:, 0]) * bag_rate
    # num_steps = entropy_data.shape[0] / total_time * filter_window
    # if do_filter:
    #     landmark_entropy_over_time_filtered = np.zeros(landmark_entropy_over_time.shape)
    #     for i in np.arange(landmark_entropy_over_time.shape[0]):
    #         if i < filter_window:
    #             landmark_entropy_over_time_filtered[i] = np.mean(landmark_entropy_over_time[:i+1])
    #         else:
    #             landmark_entropy_over_time_filtered[i] = np.mean(landmark_entropy_over_time[i-filter_window:i+1])

    #     ax[1].plot(entropy_data[:, 0], landmark_entropy_over_time_filtered, color="orange")
    # else:
    #     ax[1].plot(entropy_data[:, 0], landmark_entropy_over_time, color="orange")
    # ax[1].set_ylim([0, 300])
    # set the y axis label with the same color as the plot
    ax[1].plot(entropy_data[:, 0], landmark_entropy_over_time, color="orange")
    ax[1].set_ylabel("Sum Lmk. \n  Uncertainty", color="orange")

    ax[1].set_ylim([0, 350])
    # plot legend
    # ax[1].legend(["Sum Landmark Uncertainty"], loc="upper left", fontsize=font_size)

    # overlay the number of valid landmarks, with a separate scale and label on the right y axis
    ax2 = ax[1].twinx()
    ax2.plot(entropy_data[:, 0], entropy_data[:, 4], color="green" , linestyle="dotted")
    ax2.set_ylim([0, 30])

    # set the y axis label with the same color as the plot in dotted line
    ax2.set_ylabel("Num. Lmk.", color="green")
    ax2.yaxis.set_label_coords(1.06, 0.5)
    ax2.yaxis.label.set_rotation(45)
    # plot legend with large and boldfont
    # ax2.legend(["Num. Landmarks"], loc="center left", fontsize=font_size)


    # new plot with avg entropy per landmark
    ax[2].plot(entropy_data[:, 0], entropy_data[:, 2] / entropy_data[:, 4], color="blue")
    ax[2].set_ylabel("Avg. Lmk. \n  Uncertainty", color="blue")
    # y axis clip to 300
    ax[2].set_ylim([0, 28])
    ax2 = ax[2].twinx()
    ax2.plot(entropy_data[:, 0], entropy_data[:, 4], color="green" , linestyle="dotted")
    ax2.set_ylim([0, 25])
    ax2.set_ylabel("Num. Lmk.", color="green")
    # align y axis label to the left
    ax2.yaxis.set_label_coords(1.06, 0.5)
    ax2.yaxis.label.set_rotation(45)

    # plot ax[2] and ax2 legend together on upper left
    # ax[2].legend(["Avg. Landmark Uncertainty"], loc="upper left", fontsize=font_size)
    #plot ax2 legend below ax[2] legend
    # ax2.legend(["Num Landmarks"], loc="center left", fontsize=font_size)

    # set all ax[] label to coords (0, 0.5)
    ax[0].yaxis.set_label_coords(-0.075, 0.5)
    ax[1].yaxis.set_label_coords(-0.075 , 0.5)
    ax[2].yaxis.set_label_coords(-0.075, 0.5)    
    

    # rotate y axis for 75 degree
    ax[0].yaxis.label.set_rotation(45)
    ax[1].yaxis.label.set_rotation(45)
    ax[2].yaxis.label.set_rotation(45)


    ax[2].set_xlabel("Time (s)")
    # set x label into the plot
    ax[2].xaxis.set_label_coords(0.5, 0.25)

    # set x axis limit for all plots as the end time
    ax[0].set_xlim([0, end_time])
    ax[1].set_xlim([0, end_time])
    ax[2].set_xlim([0, end_time])
    if "closure" in filename_cur:
        # save as high resolution png
        plt.savefig(data_dir + "exploration_only_entropy_final_closure.png", bbox_inches="tight", dpi=300)
    else:
        # save as high resolution png
        plt.savefig(data_dir + "exploration_only_entropy_final_no_closure.png", bbox_inches="tight", dpi=300)

plt.show()

