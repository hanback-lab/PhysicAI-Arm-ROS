Model name inside SDF and model.config is fixed to: physicai_arm_so101

Recommended world snippet:

<include>
  <uri>model://physicai_arm_so101</uri>
  <name>physicai_arm</name>
  <pose>0 0 0 0 0 0</pose>
</include>

<joint name="physicai_arm_mount" type="fixed">
  <parent>world</parent>
  <child>physicai_arm::mount_link</child>
</joint>

Notes:
- Use the include name "physicai_arm" so the child path above stays fixed.
- Do NOT add another fixed joint to base_link at the same time.
- Asset URIs inside model.sdf are relative paths (assets/*.stl), so the outer folder name can be changed.
- Fortress plugin names are pinned to the ignition-gazebo system plugin names.

DetachableJoint topics added for pick/place:
- /grasp/object1/attach
- /grasp/object1/detach
- /grasp/object2/attach
- /grasp/object2/detach

Recommended startup step after sim begins:
- publish one detach message to both objects so they start free on the table.

Note:
- The grasp frame (gripper_frame_joint) was moved ~11 mm downward so attach can happen below the large fixed gripper collision box.
