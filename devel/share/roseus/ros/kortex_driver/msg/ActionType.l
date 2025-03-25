;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActionType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActionType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTIONTYPE")
  (make-package "KORTEX_DRIVER::ACTIONTYPE"))

(in-package "ROS")
;;//! \htmlinclude ActionType.msg.html


(intern "*UNSPECIFIED_ACTION*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*UNSPECIFIED_ACTION* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*UNSPECIFIED_ACTION* 0)
(intern "*SEND_TWIST_COMMAND*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*SEND_TWIST_COMMAND* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*SEND_TWIST_COMMAND* 1)
(intern "*SEND_WRENCH_COMMAND*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*SEND_WRENCH_COMMAND* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*SEND_WRENCH_COMMAND* 2)
(intern "*SEND_JOINT_SPEEDS*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*SEND_JOINT_SPEEDS* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*SEND_JOINT_SPEEDS* 4)
(intern "*REACH_POSE*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*REACH_POSE* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*REACH_POSE* 6)
(intern "*REACH_JOINT_ANGLES*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*REACH_JOINT_ANGLES* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*REACH_JOINT_ANGLES* 7)
(intern "*TOGGLE_ADMITTANCE_MODE*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*TOGGLE_ADMITTANCE_MODE* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*TOGGLE_ADMITTANCE_MODE* 13)
(intern "*SNAPSHOT*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*SNAPSHOT* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*SNAPSHOT* 14)
(intern "*SWITCH_CONTROL_MAPPING*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*SWITCH_CONTROL_MAPPING* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*SWITCH_CONTROL_MAPPING* 16)
(intern "*NAVIGATE_JOINTS*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*NAVIGATE_JOINTS* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*NAVIGATE_JOINTS* 17)
(intern "*NAVIGATE_MAPPINGS*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*NAVIGATE_MAPPINGS* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*NAVIGATE_MAPPINGS* 18)
(intern "*CHANGE_TWIST*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*CHANGE_TWIST* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*CHANGE_TWIST* 22)
(intern "*CHANGE_JOINT_SPEEDS*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*CHANGE_JOINT_SPEEDS* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*CHANGE_JOINT_SPEEDS* 23)
(intern "*CHANGE_WRENCH*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*CHANGE_WRENCH* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*CHANGE_WRENCH* 25)
(intern "*APPLY_EMERGENCY_STOP*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*APPLY_EMERGENCY_STOP* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*APPLY_EMERGENCY_STOP* 28)
(intern "*CLEAR_FAULTS*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*CLEAR_FAULTS* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*CLEAR_FAULTS* 29)
(intern "*TIME_DELAY*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*TIME_DELAY* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*TIME_DELAY* 31)
(intern "*EXECUTE_ACTION*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*EXECUTE_ACTION* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*EXECUTE_ACTION* 32)
(intern "*SEND_GRIPPER_COMMAND*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*SEND_GRIPPER_COMMAND* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*SEND_GRIPPER_COMMAND* 33)
(intern "*SEND_GPIO_COMMAND*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*SEND_GPIO_COMMAND* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*SEND_GPIO_COMMAND* 34)
(intern "*STOP_ACTION*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*STOP_ACTION* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*STOP_ACTION* 35)
(intern "*PLAY_PRE_COMPUTED_TRAJECTORY*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*PLAY_PRE_COMPUTED_TRAJECTORY* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*PLAY_PRE_COMPUTED_TRAJECTORY* 39)
(intern "*EXECUTE_SEQUENCE*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*EXECUTE_SEQUENCE* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*EXECUTE_SEQUENCE* 40)
(intern "*EXECUTE_WAYPOINT_LIST*" (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(shadow '*EXECUTE_WAYPOINT_LIST* (find-package "KORTEX_DRIVER::ACTIONTYPE"))
(defconstant kortex_driver::ActionType::*EXECUTE_WAYPOINT_LIST* 41)

(defun kortex_driver::ActionType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ActionType::*UNSPECIFIED_ACTION*)
        ((= const 1) 'kortex_driver::ActionType::*SEND_TWIST_COMMAND*)
        ((= const 2) 'kortex_driver::ActionType::*SEND_WRENCH_COMMAND*)
        ((= const 4) 'kortex_driver::ActionType::*SEND_JOINT_SPEEDS*)
        ((= const 6) 'kortex_driver::ActionType::*REACH_POSE*)
        ((= const 7) 'kortex_driver::ActionType::*REACH_JOINT_ANGLES*)
        ((= const 13) 'kortex_driver::ActionType::*TOGGLE_ADMITTANCE_MODE*)
        ((= const 14) 'kortex_driver::ActionType::*SNAPSHOT*)
        ((= const 16) 'kortex_driver::ActionType::*SWITCH_CONTROL_MAPPING*)
        ((= const 17) 'kortex_driver::ActionType::*NAVIGATE_JOINTS*)
        ((= const 18) 'kortex_driver::ActionType::*NAVIGATE_MAPPINGS*)
        ((= const 22) 'kortex_driver::ActionType::*CHANGE_TWIST*)
        ((= const 23) 'kortex_driver::ActionType::*CHANGE_JOINT_SPEEDS*)
        ((= const 25) 'kortex_driver::ActionType::*CHANGE_WRENCH*)
        ((= const 28) 'kortex_driver::ActionType::*APPLY_EMERGENCY_STOP*)
        ((= const 29) 'kortex_driver::ActionType::*CLEAR_FAULTS*)
        ((= const 31) 'kortex_driver::ActionType::*TIME_DELAY*)
        ((= const 32) 'kortex_driver::ActionType::*EXECUTE_ACTION*)
        ((= const 33) 'kortex_driver::ActionType::*SEND_GRIPPER_COMMAND*)
        ((= const 34) 'kortex_driver::ActionType::*SEND_GPIO_COMMAND*)
        ((= const 35) 'kortex_driver::ActionType::*STOP_ACTION*)
        ((= const 39) 'kortex_driver::ActionType::*PLAY_PRE_COMPUTED_TRAJECTORY*)
        ((= const 40) 'kortex_driver::ActionType::*EXECUTE_SEQUENCE*)
        ((= const 41) 'kortex_driver::ActionType::*EXECUTE_WAYPOINT_LIST*)
        (t nil)))

(defclass kortex_driver::ActionType
  :super ros::object
  :slots ())

(defmethod kortex_driver::ActionType
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get kortex_driver::ActionType :md5sum-) "2814fa62baddf71292e9d832616cc515")
(setf (get kortex_driver::ActionType :datatype-) "kortex_driver/ActionType")
(setf (get kortex_driver::ActionType :definition-)
      "
uint32 UNSPECIFIED_ACTION = 0

uint32 SEND_TWIST_COMMAND = 1

uint32 SEND_WRENCH_COMMAND = 2

uint32 SEND_JOINT_SPEEDS = 4

uint32 REACH_POSE = 6

uint32 REACH_JOINT_ANGLES = 7

uint32 TOGGLE_ADMITTANCE_MODE = 13

uint32 SNAPSHOT = 14

uint32 SWITCH_CONTROL_MAPPING = 16

uint32 NAVIGATE_JOINTS = 17

uint32 NAVIGATE_MAPPINGS = 18

uint32 CHANGE_TWIST = 22

uint32 CHANGE_JOINT_SPEEDS = 23

uint32 CHANGE_WRENCH = 25

uint32 APPLY_EMERGENCY_STOP = 28

uint32 CLEAR_FAULTS = 29

uint32 TIME_DELAY = 31

uint32 EXECUTE_ACTION = 32

uint32 SEND_GRIPPER_COMMAND = 33

uint32 SEND_GPIO_COMMAND = 34

uint32 STOP_ACTION = 35

uint32 PLAY_PRE_COMPUTED_TRAJECTORY = 39

uint32 EXECUTE_SEQUENCE = 40

uint32 EXECUTE_WAYPOINT_LIST = 41

")



(provide :kortex_driver/ActionType "2814fa62baddf71292e9d832616cc515")


