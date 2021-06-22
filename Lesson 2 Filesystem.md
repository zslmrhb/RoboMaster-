
# Lesson 2 - Filesystem

## ROS

**ROS, or Robotic Operating System**

## Common ROS Filesystem Commands
> **Using Linux commands will be tedious and inefficient in navigating across many ROS packages. So, that is why we will learn about the following.**

**`roscd [package name]`**
  >**moves to the specified ROS package or stack's directory**
  *Example: roscd turtlesim //moves to the turtlesim package's directory*

**`rospack [option name] [package name]`**
  >**gets information about packages**
 *Example: rospack find turtlesim  // finds the absolute path of the turtlesim package*

**`rosls [package name]`**
 >**lists the directory content of the ROS package**
 *Example: rosls turtlesim  // lists the contents in the turtlesim package directory*
 

<figure class="video_container">
  <iframe src="https://www.youtube.com/embed/enMumwvLAug" frameborder="0" allowfullscreen="true"> </iframe>
</figure>



---


##### *References*
1. [Navigating the ROS Filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
2. [ROS Robot Programming](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#books)
3. [ROS tutorial #03 Navigating ROS filesystem](https://www.youtube.com/watch?v=VkOC4UiAz_Y&list=PLk51HrKSBQ8-jTgD0qgRp1vmQeVSJ5SQC&index=3)