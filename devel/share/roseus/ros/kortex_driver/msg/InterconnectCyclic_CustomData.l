;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::InterconnectCyclic_CustomData)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'InterconnectCyclic_CustomData (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_CUSTOMDATA")
  (make-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_CUSTOMDATA"))

(in-package "ROS")
;;//! \htmlinclude InterconnectCyclic_CustomData.msg.html


(defclass kortex_driver::InterconnectCyclic_CustomData
  :super ros::object
  :slots (_custom_data_id _custom_data_0 _custom_data_1 _custom_data_2 _custom_data_3 _custom_data_4 _custom_data_5 _custom_data_6 _custom_data_7 _custom_data_8 _custom_data_9 _custom_data_10 _custom_data_11 _custom_data_12 _custom_data_13 _custom_data_14 _custom_data_15 _oneof_tool_customData ))

(defmethod kortex_driver::InterconnectCyclic_CustomData
  (:init
   (&key
    ((:custom_data_id __custom_data_id) (instance kortex_driver::InterconnectCyclic_MessageId :init))
    ((:custom_data_0 __custom_data_0) 0)
    ((:custom_data_1 __custom_data_1) 0)
    ((:custom_data_2 __custom_data_2) 0)
    ((:custom_data_3 __custom_data_3) 0)
    ((:custom_data_4 __custom_data_4) 0)
    ((:custom_data_5 __custom_data_5) 0)
    ((:custom_data_6 __custom_data_6) 0)
    ((:custom_data_7 __custom_data_7) 0)
    ((:custom_data_8 __custom_data_8) 0)
    ((:custom_data_9 __custom_data_9) 0)
    ((:custom_data_10 __custom_data_10) 0)
    ((:custom_data_11 __custom_data_11) 0)
    ((:custom_data_12 __custom_data_12) 0)
    ((:custom_data_13 __custom_data_13) 0)
    ((:custom_data_14 __custom_data_14) 0)
    ((:custom_data_15 __custom_data_15) 0)
    ((:oneof_tool_customData __oneof_tool_customData) (instance kortex_driver::InterconnectCyclic_CustomData_tool_customData :init))
    )
   (send-super :init)
   (setq _custom_data_id __custom_data_id)
   (setq _custom_data_0 (round __custom_data_0))
   (setq _custom_data_1 (round __custom_data_1))
   (setq _custom_data_2 (round __custom_data_2))
   (setq _custom_data_3 (round __custom_data_3))
   (setq _custom_data_4 (round __custom_data_4))
   (setq _custom_data_5 (round __custom_data_5))
   (setq _custom_data_6 (round __custom_data_6))
   (setq _custom_data_7 (round __custom_data_7))
   (setq _custom_data_8 (round __custom_data_8))
   (setq _custom_data_9 (round __custom_data_9))
   (setq _custom_data_10 (round __custom_data_10))
   (setq _custom_data_11 (round __custom_data_11))
   (setq _custom_data_12 (round __custom_data_12))
   (setq _custom_data_13 (round __custom_data_13))
   (setq _custom_data_14 (round __custom_data_14))
   (setq _custom_data_15 (round __custom_data_15))
   (setq _oneof_tool_customData __oneof_tool_customData)
   self)
  (:custom_data_id
   (&rest __custom_data_id)
   (if (keywordp (car __custom_data_id))
       (send* _custom_data_id __custom_data_id)
     (progn
       (if __custom_data_id (setq _custom_data_id (car __custom_data_id)))
       _custom_data_id)))
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
  (:custom_data_8
   (&optional __custom_data_8)
   (if __custom_data_8 (setq _custom_data_8 __custom_data_8)) _custom_data_8)
  (:custom_data_9
   (&optional __custom_data_9)
   (if __custom_data_9 (setq _custom_data_9 __custom_data_9)) _custom_data_9)
  (:custom_data_10
   (&optional __custom_data_10)
   (if __custom_data_10 (setq _custom_data_10 __custom_data_10)) _custom_data_10)
  (:custom_data_11
   (&optional __custom_data_11)
   (if __custom_data_11 (setq _custom_data_11 __custom_data_11)) _custom_data_11)
  (:custom_data_12
   (&optional __custom_data_12)
   (if __custom_data_12 (setq _custom_data_12 __custom_data_12)) _custom_data_12)
  (:custom_data_13
   (&optional __custom_data_13)
   (if __custom_data_13 (setq _custom_data_13 __custom_data_13)) _custom_data_13)
  (:custom_data_14
   (&optional __custom_data_14)
   (if __custom_data_14 (setq _custom_data_14 __custom_data_14)) _custom_data_14)
  (:custom_data_15
   (&optional __custom_data_15)
   (if __custom_data_15 (setq _custom_data_15 __custom_data_15)) _custom_data_15)
  (:oneof_tool_customData
   (&rest __oneof_tool_customData)
   (if (keywordp (car __oneof_tool_customData))
       (send* _oneof_tool_customData __oneof_tool_customData)
     (progn
       (if __oneof_tool_customData (setq _oneof_tool_customData (car __oneof_tool_customData)))
       _oneof_tool_customData)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/InterconnectCyclic_MessageId _custom_data_id
    (send _custom_data_id :serialization-length)
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
    ;; uint32 _custom_data_8
    4
    ;; uint32 _custom_data_9
    4
    ;; uint32 _custom_data_10
    4
    ;; uint32 _custom_data_11
    4
    ;; uint32 _custom_data_12
    4
    ;; uint32 _custom_data_13
    4
    ;; uint32 _custom_data_14
    4
    ;; uint32 _custom_data_15
    4
    ;; kortex_driver/InterconnectCyclic_CustomData_tool_customData _oneof_tool_customData
    (send _oneof_tool_customData :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/InterconnectCyclic_MessageId _custom_data_id
       (send _custom_data_id :serialize s)
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
     ;; uint32 _custom_data_8
       (write-long _custom_data_8 s)
     ;; uint32 _custom_data_9
       (write-long _custom_data_9 s)
     ;; uint32 _custom_data_10
       (write-long _custom_data_10 s)
     ;; uint32 _custom_data_11
       (write-long _custom_data_11 s)
     ;; uint32 _custom_data_12
       (write-long _custom_data_12 s)
     ;; uint32 _custom_data_13
       (write-long _custom_data_13 s)
     ;; uint32 _custom_data_14
       (write-long _custom_data_14 s)
     ;; uint32 _custom_data_15
       (write-long _custom_data_15 s)
     ;; kortex_driver/InterconnectCyclic_CustomData_tool_customData _oneof_tool_customData
       (send _oneof_tool_customData :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/InterconnectCyclic_MessageId _custom_data_id
     (send _custom_data_id :deserialize buf ptr-) (incf ptr- (send _custom_data_id :serialization-length))
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
   ;; uint32 _custom_data_8
     (setq _custom_data_8 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_9
     (setq _custom_data_9 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_10
     (setq _custom_data_10 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_11
     (setq _custom_data_11 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_12
     (setq _custom_data_12 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_13
     (setq _custom_data_13 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_14
     (setq _custom_data_14 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _custom_data_15
     (setq _custom_data_15 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/InterconnectCyclic_CustomData_tool_customData _oneof_tool_customData
     (send _oneof_tool_customData :deserialize buf ptr-) (incf ptr- (send _oneof_tool_customData :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::InterconnectCyclic_CustomData :md5sum-) "4f180448c3bd7285547e1bae08efc80a")
(setf (get kortex_driver::InterconnectCyclic_CustomData :datatype-) "kortex_driver/InterconnectCyclic_CustomData")
(setf (get kortex_driver::InterconnectCyclic_CustomData :definition-)
      "
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



(provide :kortex_driver/InterconnectCyclic_CustomData "4f180448c3bd7285547e1bae08efc80a")


