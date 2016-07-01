#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-20
# PURPOSE: Make it easier to run YuMi scripts - Run YuMi demo in RViz

# Initialize Variables
runCommand="roslaunch yumi_moveit_config demo.launch"; # set initial command for starting robot interface
flag_argError=false; # create flag to indicate if input arguments were valid
flag_twoGrippers=false; # flag to indicate if two grippers argument has already been set
flag_rviz=false; # flag to indicate if state servers only argument has already been set

# Iterate Through Provided Arguments
for argument in "$@"; do # for all provided arguments
	if [ "$argument" == "rviz" ]; then # if the user would like to only run the state servers
		if [ $flag_rviz = false ]; then # if this argument has not been set yet
			echo "Loading RViz on execution." # notify the user that the argument has been received
			runCommand="$runCommand rviz:=true"; # add argument to the run command
			flag_rviz=true; # indicate that the argument has been set
		else # if the argument has already been set
			echo "Already set the argument for state_servers_only." # notify the user that this argument has already been set
		fi
	elif [ "$argument" == "two_grippers" ]; then # if the user is using two gripperson YuMi
		if [ $flag_twoGrippers = false ]; then # if this argument has not been set yet
			echo "Assuming using two grippers." # notify the user that the argument has been received
			runCommand="$runCommand two_grippers:=true"; # add argument to the run command
			flag_twoGrippers=true; # indicate that the argument has been set
		else # if the argument has already been set
			echo "Already set the argument for two_grippers." # notify the user that this argument has already been set
		fi
	else # if the argument is not recognized
		echo "Argument $currentArgument is not recognized." # notify the user that the current argument is not recognized
		flag_argError=true; # indicate that an argument error has occurred
		break; # break from the loop
	fi
done

# Check if All Arguments Were Valid
if [ $flag_argError = false ]; then # if all arguments were valid
	sleep 2; # sleep to allow the user to see the terminal echo's
	$runCommand # run the robot interface node initializer command with user desired arguments
else # if one or more arguments were not valid
	echo "Error occurred. Not loading YuMi demo due to error." # notify user that the robot interface will not be executed
fi


