# joy-ctrl

This node is to support joystick teleoperation of the AUV in ROV mode. This 

## To execute
```bash
roslaunch v2_control joy_teleop.launch
```

This automatically launches `joy` node for interfacing between the computer and the joystick, and the `joy_control.cpp` node which would convert the inputs from the joystick into command velocity values for the gazebo to decipher.

## Understanding implementation of stoppable and reusable threads
The following snippet forms the core that is responsible for:
- Executing PID controller on a separate thread; continuously without any interrupts
- Stop the background thread
- Re-start the background thread

This snippet has been taken from `joy_control.cpp` file. Rest of the code may differ from application to application but for continuos turning ON and OFF of PID controller, the following snippet is a must.
```
...
...
int main()
{
  ...
  ...
  
  /*this same thread object will be used to create a thread where the callable is the same*/

  std::thread pid_thread;

  // do_not_quit_ is set/reset by user
  while (do_not_quit_ && ros::ok())
  {
    if (v2_.is_traversing_)
    {
	  /* execute if the thread object is not joinable =>
	  	thread previous thread that was created using this
		object is terminated/killed.
	  */
      if(!pid_thread.joinable())
      {
		/* new thread assigned to our old 
		thread object using lambda expression
		*/
        pid_thread = std::thread([&](){
		  
		  /* reuseThread is inherited function 
		  from StoppableThread. This function creates a
		  new future_obj using new exit_signal_obj
		  that helps in creating stoppable  thread.
		  Read StoppableThread class to learn more.
		  */
          v2_.reuseThread();

		  /* run is inherited from UWV and was 
		  virtual in StoppableThread. This 
		  executes the PID loop in the background.
		  */
          v2_.run();
        });
      }
    }
    else
    {
	  /* stopRequested is inherited from StoppableThread.
	  Once the value is True, it remains true until
	  we create a new future_obj using new exit_signal_obj.
	  */
      if(!v2_.stopRequested())
      {
		/* Executing this would send a signal that would
		eventually end the PID thread in the background.
		*/
        v2_.stop();
      }

	  /* Joins the background thread with the
	  current thread so that it can terminate.
	  */
      if(pid_thread.joinable())
      {
        pid_thread.join();
      }
    }
    ros::spinOnce();
  }

  /* Closing the background thread at the end of the 
  application.
  */
  if(!v2_.stopRequested()){
    v2_.stop();
  }
  if(pid_thread.joinable()){
    pid_thread.join();
  }
  ROS_INFO_STREAM("Sayonara.");

  return 0;
}
...
...

```