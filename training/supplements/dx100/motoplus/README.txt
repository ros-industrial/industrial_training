Due to the limitations of the Motoplus IDE, the source files for the Motoplus application must be stored in the individual project directories.  Previously svn externals were used to make this transparent to the developer.  This caused problems when branching and tagging.  It also caused extremely long check out times.  Because of this, svn externals were removed.  In their place windows batch scripts have been created that mimic this syncronization, albeit manual.

In the motoros_lib directory the following files are used:
motoplus_sync_script_generator.xlsm - used to auto-generate sync scripts below
SyncFromSource.bat - syncs local copies of source/header files FROM the source location (THIS SHOULD BE RAN FIRST)
SyncToSource.bat - Syncs local source/header files TO the source location (THIS SHOULD BE RAN LAST)

In the motoros_server directory the following files are used:
robot_sync_script_generator.xlsm - used to auto-generate sync scripts for robot-server executable
SyncFromSource.bat - syncs local copies from the motros_lib.  NOTE: there is no need to sync back to source since the source files are generated in the motoros_lib project (i.e. any changes should be made there and then synced to the robot specific project).


WARNING: THIS METHOD OF SYNCRONIZATION REQUIRES THE USER TO MANUALY SYNC FILES BACK TO THEIR SOURCE LOCATIONS AND THEN CHECK THEM IN.


