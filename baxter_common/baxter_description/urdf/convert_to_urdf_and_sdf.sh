roscd baxter_description/urdf
xacro --inorder left_end_effector.urdf.xacro gazebo:=true > left_end_effector.urdf
gz sdf --print left_end_effector.urdf > left_end_effector.sdf
