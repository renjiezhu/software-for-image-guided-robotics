#!usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Time
import time
import matplotlib.pyplot as plt



# subscriber callbacks
def callback(returnMsg1):
    print('Back info reveived')
    timeCurrent = rospy.Time.now()
    Latency1 = timeCurrent.to_sec() - returnMsg1.data.to_sec()
    print("latency = " + str(Latency1))
    BBB1Latency.append(Latency1)


if __name__ == "__main__":
    
    # Initialize Latency Lists #
    BBB1Latency = []


    rospy.init_node('SnakeControl', anonymous=True)
    
    # Set up publishers #
    pub = rospy.Publisher('latency_test_out', Time, queue_size=1000)

    # Set up subscriber #
    sub = rospy.Subscriber('latency_test_back', Time, callback)


    count = 1

    # send out 30 pings
    while count<200:
    # for x in range(30):
        timeOutgoing = rospy.Time.now()

        BBB1_outgoing = Time()
        BBB1_outgoing.data = timeOutgoing
        pub.publish(BBB1_outgoing)

        
        # print("Current Cycle: " + str(x))
        print("Current Cycle: " + str(count))
        time.sleep(0.1)

        count += 1


    print("Latency mean = " + str(np.mean(BBB1Latency)) + " stdev = " + str(np.std(BBB1Latency)))


    data_to_plot = [BBB1Latency]

    fig1, ax1 = plt.subplots()
    ax1.set_title('Basic Plot')
    ax1.boxplot(data_to_plot)
    plt.show()

    print("Overall mean Latency = " + str((np.mean(BBB1Latency)+np.mean(BBB2Latency)+np.mean(BBB3Latency)+np.mean(BBB4Latency))/4*10000) + "micro s")

