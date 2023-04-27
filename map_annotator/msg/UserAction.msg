string CREATE=create
string DELETE=delete
string GOTO=goto
string RENAME=rename
string RELOCATE=relocate

string command
string name # The name of the pose the command applies to
string updated_name # If command is RENAME, this is the new name of the pose
geometry_msgs/Pose updated_pose # if command is RELOCATE, relocate the marker