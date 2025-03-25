;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::InterconnectCyclic_CustomData_tool_customData)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'InterconnectCyclic_CustomData_tool_customData (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_CUSTOMDATA_TOOL_CUSTOMDATA")
  (make-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_CUSTOMDATA_TOOL_CUSTOMDATA"))

(in-package "ROS")
;;//! \htmlinclude InterconnectCyclic_CustomData_tool_customData.msg.html


(defclass kortex_driver::InterconnectCyclic_CustomData_tool_customData
  :super ros::object
  :slots (_gripper_custom_data ))

(defmethod kortex_driver::InterconnectCyclic_CustomData_tool_customData
  (:init
   (&key
    ((:gripper_custom_data __gripper_custom_data) ())
    )
   (send-super :init)
   (setq _gripper_custom_data __gripper_custom_data)
   self)
  (:gripper_custom_data
   (&rest __gripper_custom_data)
   (if (keywordp (car __gripper_custom_data))
       (send* _gripper_custom_data __gripper_custom_data)
     (progn
       (if __gripper_custom_data (setq _gripper_custom_data (car __gripper_custom_data)))
       _gripper_custom_data)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/GripperCyclic_CustomData[] _gripper_custom_data
    (apply #'+ (send-all _gripper_custom_data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GripperCyclic_CustomData[] _gripper_custom_data
     (write-long (length _gripper_custom_data) s)
     (dolist (elem _gripper_custom_data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GripperCyclic_CustomData[] _gripper_custom_data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gripper_custom_data (let (r) (dotimes (i n) (push (instance kortex_driver::GripperCyclic_CustomData :init) r)) r))
     (dolist (elem- _gripper_custom_data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::InterconnectCyclic_CustomData_tool_customData :md5sum-) "08300f61864775cfea0ba45676ef4872")
(setf (get kortex_driver::InterconnectCyclic_CustomData_tool_customData :datatype-) "kortex_driver/InterconnectCyclic_CustomData_tool_customData")
(setf (get kortex_driver::InterconnectCyclic_CustomData_tool_customData :definition-)
      "
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



(provide :kortex_driver/InterconnectCyclic_CustomData_tool_customData "08300f61864775cfea0ba45676ef4872")


