# catkin_make -C /home/$USER/dev_ws ;
# source /home/$USER/dev_ws/devel/setup.bash


# Get the current directory
CURR_DIR=$(pwd)
# Get the location of the viral package
roscd mloam
PACKAGE_DIR=$(pwd)
# Return to the current dir, print the directions
cd $CURR_DIR
echo CURRENT DIR: $CURR_DIR
echo VIRAL DIR:   $PACKAGE_DIR

export EPOC_DIR=/home/$USER/nucssd2/MATLAB_WS/RAL_edits/mloam/NTUVIRAL_2lidar_nuc10
export DATASET_LOCATION=/home/$USER/DATASETS/NTU_VIRAL/

export CAPTURE_SCREEN=false;
export LOG_DATA=true;

#region 0 UWB NO VIS --------------------------------------------------------------------------------------------------

wait;
./run_one_bag_clementi.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_01 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
wait;
./run_one_bag_clementi.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_02 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
wait;
./run_one_bag_clementi.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR eee_03 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;

wait;
./run_one_bag_clementi.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_01 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
wait;
./run_one_bag_clementi.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_02 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
wait;
./run_one_bag_clementi.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR nya_03 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;

wait;
./run_one_bag_clementi.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_01 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
wait;
./run_one_bag_clementi.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_02 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
wait;
./run_one_bag_clementi.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR sbs_03 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;

#endregion NO UWB NO VIS ----------------------------------------------------------------------------------------------


#region ## Poweroff ---------------------------------------------------------------------------------------------------

wait;
# poweroff

#endregion ## Poweroff ------------------------------------------------------------------------------------------------

