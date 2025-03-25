;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GripperCyclic_CustomData)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GripperCyclic_CustomData (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GRIPPERCYCLIC_CUSTOMDATA")
  (make-package "KORTEX_DRIVER::GRIPPERCYCLIC_CUSTOMDATA"))

(in-package "ROS")
;;//! \htmlinclude GripperCyclic_CustomData.msg.html


(defclass kortex_driver::GripperCyclic_CustomData
  :super ros::object
  :slots (_custom_data_id _gripper_custom_data _motor_custom_data ))

(defmethod kortex_driver::GripperCyclic_CustomData
  (:init
   (&key
    ((:custom_data_id __custom_data_id) (instance kortex_driver::GripperCyclic_MessageId :init))
    ((:gripper_custom_data __gripper_custom_data) (instance kortex_driver::CustomDataUnit :init))
    ((:motor_custom_data __motor_custom_data) ())
    )
   (send-super :init)
   (setq _custom_data_id __custom_data_id)
   (setq _gripper_custom_data __gripper_custom_data)
   (setq _motor_custom_data __motor_custom_data)
   self)
  (:custom_data_id
   (&rest __custom_data_id)
   (if (keywordp (car __custom_data_id))
       (send* _custom_data_id __custom_data_id)
     (progn
       (if __custom_data_id (setq _custom_data_id (car __custom_data_id)))
       _custom_data_id)))
  (:gripper_custom_data
   (&rest __gripper_custom_data)
   (if (keywordp (car __gripper_custom_data))
       (send* _gripper_custom_data __gripper_custom_data)
     (progn
       (if __gripper_custom_data (setq _gripper_custom_data (car __gripper_custom_data)))
       _gripper_custom_data)))
  (:motor_custom_data
   (&rest __motor_custom_data)
   (if (keywordp (car __motor_custom_data))
       (send* _motor_custom_data __motor_custom_data)
     (progn
       (if __motor_custom_data (setq _motor_custom_data (car __motor_custom_data)))
       _motor_custom_data)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/GripperCyclic_MessageId _custom_data_id
    (send _custom_data_id :serialization-length)
    ;; kortex_driver/CustomDataUnit _gripper_custom_data
    (send _gripper_custom_data :serialization-length)
    ;; kortex_driver/CustomDataUnit[] _motor_custom_data
    (apply #'+ (send-all _motor_custom_data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GripperCyclic_MessageId _custom_data_id
       (send _custom_data_id :serialize s)
     ;; kortex_driver/CustomDataUnit _gripper_custom_data
       (send _gripper_custom_data :serialize s)
     ;; kortex_driver/CustomDataUnit[] _motor_custom_data
     (write-long (length _motor_custom_data) s)
     (dolist (elem _motor_custom_data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GripperCyclic_MessageId _custom_data_id
     (send _custom_data_id :deserialize buf ptr-) (incf ptr- (send _custom_data_id :serialization-length))
   ;; kortex_driver/CustomDataUnit _gripper_custom_data
     (send _gripper_custom_data :deserialize buf ptr-) (incf ptr- (send _gripper_custom_data :serialization-length))
   ;; kortex_driver/CustomDataUnit[] _motor_custom_data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _motor_custom_data (let (r) (dotimes (i n) (push (instance kortex_driver::CustomDataUnit :init) r)) r))
     (dolist (elem- _motor_custom_data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::GripperCyclic_CustomData :md5sum-) "ccb37c7f71598724d847a97b3a0c62f5")
(setf (get kortex_driver::GripperCyclic_CustomData :datatype-) "kortex_driver/GripperCyclic_CustomData")
(setf (get kortex_driver::GripperCyclic_CustomData :definition-)
      "
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



(provide :kortex_driver/GripperCyclic_CustomData "ccb37c7f71598724d847a97b3a0c62f5")


