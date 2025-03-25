; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BackupEvent.msg.html

(cl:defclass <BackupEvent> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass BackupEvent (<BackupEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BackupEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BackupEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BackupEvent> is deprecated: use kortex_driver-msg:BackupEvent instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<BackupEvent>)))
    "Constants for message type '<BackupEvent>"
  '((:UNSPECIFIED_BACKUP_EVENT . 0)
    (:BACKUP_RESTORED . 1)
    (:BACKUP_UPLOADED . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'BackupEvent)))
    "Constants for message type 'BackupEvent"
  '((:UNSPECIFIED_BACKUP_EVENT . 0)
    (:BACKUP_RESTORED . 1)
    (:BACKUP_UPLOADED . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BackupEvent>) ostream)
  "Serializes a message object of type '<BackupEvent>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BackupEvent>) istream)
  "Deserializes a message object of type '<BackupEvent>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BackupEvent>)))
  "Returns string type for a message object of type '<BackupEvent>"
  "kortex_driver/BackupEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BackupEvent)))
  "Returns string type for a message object of type 'BackupEvent"
  "kortex_driver/BackupEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BackupEvent>)))
  "Returns md5sum for a message object of type '<BackupEvent>"
  "a96436414fbd5b9a597d2aa04b026d71")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BackupEvent)))
  "Returns md5sum for a message object of type 'BackupEvent"
  "a96436414fbd5b9a597d2aa04b026d71")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BackupEvent>)))
  "Returns full string definition for message of type '<BackupEvent>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_BACKUP_EVENT = 0~%~%uint32 BACKUP_RESTORED = 1~%~%uint32 BACKUP_UPLOADED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BackupEvent)))
  "Returns full string definition for message of type 'BackupEvent"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_BACKUP_EVENT = 0~%~%uint32 BACKUP_RESTORED = 1~%~%uint32 BACKUP_UPLOADED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BackupEvent>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BackupEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'BackupEvent
))
