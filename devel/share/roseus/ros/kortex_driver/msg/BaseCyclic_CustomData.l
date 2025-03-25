;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BaseCyclic_CustomData)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BaseCyclic_CustomData (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASECYCLIC_CUSTOMDATA")
  (make-package "KORTEX_DRIVER::BASECYCLIC_CUSTOMDATA"))

(in-package "ROS")
;;//! \htmlinclude BaseCyclic_CustomData.msg.html


(defclass kortex_driver::BaseCyclic_CustomData
  :super ros::object
  :slots (_frame_id _custom_data_0 _custom_data_1 _custom_data_2 _custom_data_3 _custom_data_4 _custom_data_5 _custom_data_6 _custom_data_7 _actuators_custom_data _interconnect_custom_data ))

(defmethod kortex_driver::BaseCyclic_CustomData
  (:init
   (&key
    ((:frame_id __frame_id) 0)
    ((:custom_data_0 __custom_data_0) 0)
    ((:custom_data_1 __custom_data_1) 0)
    ((:custom_data_2 __custom_data_2) 0)
    ((:custom_data_3 __custom_data_3) 0)
    ((:custom_data_4 __custom_data_4) 0)
    ((:custom_data_5 __custom_data_5) 0)
    ((:custom_data_6 __custom_data_6) 0)
    ((:custom_data_7 __custom_data_7) 0)
    ((:actuators_custom_data __actuators_custom_data) ())
    ((:interconnect_custom_data __interconnect_custom_data) (instance kortex_driver::InterconnectCyclic_CustomData :init))
    )
   (send-super :init)
   (setq _frame_id (round __frame_id))
   (setq _custom_data_0 (round __custom_data_0))
   (setq _custom_data_1 (round __custom_data_1))
   (setq _custom_data_2 (round __custom_data_2))
   (setq _custom_data_3 (round __custom_data_3))
   (setq _custom_data_4 (round __custom_data_4))
   (setq _custom_data_5 (round __custom_data_5))
   (setq _custom_data_6 (round __custom_data_6))
   (setq _custom_data_7 (round __custom_data_7))
   (setq _actuators_custom_data __actuators_custom_data)
   (setq _interconnect_custom_data __interconnect_custom_data)
   self)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:custom_data_0
   (&optional __custom_data_0)
   (if __custom_data_0 (setq _custom_data_0 __custom_data_0)) _custom_data_0)
  (:custom_data_1
   (&optional __custom_data_1)
   (if __custom_data_1 (setq _custom_data_1 __custom_data_1)) _custom_data_1)
  (:custom_data_2
   (&optional __custom_data_2)
   (if __custom_data_2 (setq _custom_data_2 __custom_data_2)) _custom_data_2)
  (:custom_data_3
   (&optional __custom_data_3)
   (if __custom_data_3 (setq _custom_data_3 __custom_data_3)) _custom_data_3)
  (:custom_data_4
   (&optional __custom_data_4)
   (if __custom_data_4 (setq _custom_data_4 __custom_data_4)) _custom_data_4)
  (:custom_data_5
   (&optional __custom_data_5)
   (if __custom_data_5 (setq _custom_data_5 __custom_data_5)) _custom_data_5)
  (:custom_data_6
   (&optional __custom_data_6)
   (if __custom_data_6 (setq _custom_data_6 __custom_data_6)) _custom_data_6)
  (:custom_data_7
   (&optional __custom_data_7)
   (if __custom_data_7 (setq _custom_data_7 __custom_data_7)) _custom_data_7)
  (:actuators_custom_data
   (&rest __actuators_custom_data)
   (if (keywordp (car __actuators_custom_data))
       (send* _actuators_custom_data __actuators_custom_data)
     (progn
       (if __actuators_custom_data (setq _actuators_custom_data (car __actuators_custom_data)))
       _actuators_custom_data)))
  (:interconnect_custom_data
   (&rest __interconnect_custom_data)
   (if (keywordp (car __interconnect_custom_data))
       (send* _interconnect_custom_data __interconnect_custom_data)
     (progn
       (if __interconnect_custom_data (setq _interconnect_custom_data (car __interconnect_custom_data)))
       _interconnect_custom_data)))
  (:serialization-length
   ()
   (+
    ;; uint32 _frame_id
    4
    ;; uint32 _custom_data_0
    4
    ;; uint32 _custom_data_1
    4
    ;; uint32 _custom_data_2
    4
    ;; uint32 _custom_data_3
    4
    ;; uint32 _custom_data_4
    4
    ;; uint32 _custom_data_5
    4
    ;; uint32 _custom_data_6
    4
    ;; uint32 _custom_data_7
    4
    ;; kortex_driver/ActuatorCustomData[] _actuators_custom_data
    (apply #'+ (send-all _actuators_custom_data :serialization-length)) 4
    ;; kortex_driver/InterconnectCyclic_CustomData _interconnect_custom_data
    (send _interconnect_custom_data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _frame_id
       (write-long _frame_id s)
     ;; uint32 _custom_data_0
       (write-long _custom_data_0 s)
     ;; uint32 _custom_data_1
       (write-long _custom_data_1 s)
     ;; uint32 _custom_data_2
       (write-long _custom_data_2 s)
     ;; uint32 _custom_data_3
       (write-long _custom_data_3 s)
     ;; uint32 _custom_data_4
       (write-long _custom_data_4 s)
     ;; uint32 _custom_data_5
       (write-long _custom_data_5 s)
     ;; uint32 _custom_data_6
       (write-long _custom_data_6 s)
     ;; uint32 _custom_data_7
       (write-long _custom_data_7 s)
     ;; kortex_driver/ActuatorCustomData[] _actuators_custom_data
     (write-long (length _actuators_custom_data) s)
     (dolist (elem _actuators_custom_data)
       (send elem :serialize s)
       )
     ;; kortex_driver/InterconnectCyclic_CustomData _interconnect_custom_data
       (send _interconnect_custom_data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _frame_id
     (setq _frame_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_0
     (setq _custom_data_0 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_1
     (setq _custom_data_1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_2
     (setq _custom_data_2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_3
     (setq _custom_data_3 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_4
     (setq _custom_data_4 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_5
     (setq _custom_data_5 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_6
     (setq _custom_data_6 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_7
     (setq _custom_data_7 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/ActuatorCustomData[] _actuators_custom_data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _actuators_custom_data (let (r) (dotimes (i n) (push (instance kortex_driver::ActuatorCustomData :init) r)) r))
     (dolist (elem- _actuators_custom_data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/InterconnectCyclic_CustomData _interconnect_custom_data
     (send _interconnect_custom_data :deserialize buf ptr-) (incf ptr- (send _interconnect_custom_data :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::BaseCyclic_CustomData :md5sum-) "de86e2c250eb6e05b62cbbd55c2a087b")
(setf (get kortex_driver::BaseCyclic_CustomData :datatype-) "kortex_driver/BaseCyclic_CustomData")
(setf (get kortex_driver::BaseCyclic_CustomData :definition-)
      "
uint32 frame_id
uint32 custom_data_0
uint32 custom_data_1
uint32 custom_data_2
uint32 custom_data_3
uint32 custom_data_4
uint32 custom_data_5
uint32 custom_data_6
uint32 custom_data_7
ActuatorCustomData[] actuators_custom_data
InterconnectCyclic_CustomData interconnect_custom_data
================================================================================
MSG: kortex_driver/ActuatorCustomData

uint32 command_id
uint32 custom_data_0
uint32 custom_data_1
uint32 custom_data_2
uint32 custom_data_3
uint32 custom_data_4
uint32 custom_data_5
uint32 custom_data_6
uint32 custom_data_7
uint32 custom_data_8
uint32 custom_data_9
uint32 custom_data_10
uint32 custom_data_11
uint32 custom_data_12
uint32 custom_data_13
uint32 custom_data_14
uint32 custom_data_15
================================================================================
MSG: kortex_driver/InterconnectCyclic_CustomData

InterconnectCyclic_MessageId custom_data_id
uint32 custom_data_0
uint32 custom_data_1
uint32 custom_data_2
uint32 custom_data_3
uint32 custom_data_4
uint32 custom_data_5
uint32 custom_data_6
uint32 custom_data_7
uint32 custom_data_8
uint32 custom_data_9
uint32 custom_data_10
uint32 custom_data_11
uint32 custom_data_12
uint32 custom_data_13
uint32 custom_data_14
uint32 custom_data_15
InterconnectCyclic_CustomData_tool_customData oneof_tool_customData
================================================================================
MSG: kortex_driver/InterconnectCyclic_MessageId

uint32 identifier
================================================================================
MSG: kortex_driver/InterconnectCyclic_CustomData_tool_customData

GripperCyclic_CustomData[] gripper_custom_data
================================================================================
MSG: kortex_driver/GripperCyclic_CustomData

GripperCyclic_MessageId custom_data_id
CustomDataUnit gripper_custom_data
CustomDataUnit[] motor_custom_data
================================================================================
MSG: kortex_driver/GripperCyclic_MessageId

uint32 identifier
================================================================================
MSG: kortex_driver/CustomDataUnit

uint32 custom_data_0
uint32 custom_data_1
uint32 custom_data_2
uint32 custom_data_3
uint32 custom_data_4
uint32 custom_data_5
uint32 custom_data_6
uint32 custom_data_7
uint32 custom_data_8
uint32 custom_data_9
uint32 custom_data_10
uint32 custom_data_11
uint32 custom_data_12
uint32 custom_data_13
uint32 custom_data_14
uint32 custom_data_15
")



(provide :kortex_driver/BaseCyclic_CustomData "de86e2c250eb6e05b62cbbd55c2a087b")


