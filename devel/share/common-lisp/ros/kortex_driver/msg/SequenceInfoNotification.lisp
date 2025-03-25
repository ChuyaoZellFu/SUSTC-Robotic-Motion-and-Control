; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SequenceInfoNotification.msg.html

(cl:defclass <SequenceInfoNotification> (roslisp-msg-protocol:ros-message)
  ((event_identifier
    :reader event_identifier
    :initarg :event_identifier
    :type cl:integer
    :initform 0)
   (sequence_handle
    :reader sequence_handle
    :initarg :sequence_handle
    :type kortex_driver-msg:SequenceHandle
    :initform (cl:make-instance 'kortex_driver-msg:SequenceHandle))
   (task_index
    :reader task_index
    :initarg :task_index
    :type cl:integer
    :initform 0)
   (group_identifier
    :reader group_identifier
    :initarg :group_identifier
    :type cl:integer
    :initform 0)
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type kortex_driver-msg:Timestamp
    :initform (cl:make-instance 'kortex_driver-msg:Timestamp))
   (user_handle
    :reader user_handle
    :initarg :user_handle
    :type kortex_driver-msg:UserProfileHandle
    :initform (cl:make-instance 'kortex_driver-msg:UserProfileHandle))
   (abort_details
    :reader abort_details
    :initarg :abort_details
    :type cl:integer
    :initform 0)
   (connection
    :reader connection
    :initarg :connection
    :type kortex_driver-msg:Connection
    :initform (cl:make-instance 'kortex_driver-msg:Connection)))
)

(cl:defclass SequenceInfoNotification (<SequenceInfoNotification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SequenceInfoNotification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SequenceInfoNotification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SequenceInfoNotification> is deprecated: use kortex_driver-msg:SequenceInfoNotification instead.")))

(cl:ensure-generic-function 'event_identifier-val :lambda-list '(m))
(cl:defmethod event_identifier-val ((m <SequenceInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:event_identifier-val is deprecated.  Use kortex_driver-msg:event_identifier instead.")
  (event_identifier m))

(cl:ensure-generic-function 'sequence_handle-val :lambda-list '(m))
(cl:defmethod sequence_handle-val ((m <SequenceInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sequence_handle-val is deprecated.  Use kortex_driver-msg:sequence_handle instead.")
  (sequence_handle m))

(cl:ensure-generic-function 'task_index-val :lambda-list '(m))
(cl:defmethod task_index-val ((m <SequenceInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:task_index-val is deprecated.  Use kortex_driver-msg:task_index instead.")
  (task_index m))

(cl:ensure-generic-function 'group_identifier-val :lambda-list '(m))
(cl:defmethod group_identifier-val ((m <SequenceInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:group_identifier-val is deprecated.  Use kortex_driver-msg:group_identifier instead.")
  (group_identifier m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <SequenceInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timestamp-val is deprecated.  Use kortex_driver-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'user_handle-val :lambda-list '(m))
(cl:defmethod user_handle-val ((m <SequenceInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_handle-val is deprecated.  Use kortex_driver-msg:user_handle instead.")
  (user_handle m))

(cl:ensure-generic-function 'abort_details-val :lambda-list '(m))
(cl:defmethod abort_details-val ((m <SequenceInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:abort_details-val is deprecated.  Use kortex_driver-msg:abort_details instead.")
  (abort_details m))

(cl:ensure-generic-function 'connection-val :lambda-list '(m))
(cl:defmethod connection-val ((m <SequenceInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connection-val is deprecated.  Use kortex_driver-msg:connection instead.")
  (connection m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SequenceInfoNotification>) ostream)
  "Serializes a message object of type '<SequenceInfoNotification>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event_identifier)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sequence_handle) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'group_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'group_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'group_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'group_identifier)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'timestamp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'user_handle) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abort_details)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'abort_details)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'abort_details)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'abort_details)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'connection) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SequenceInfoNotification>) istream)
  "Deserializes a message object of type '<SequenceInfoNotification>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event_identifier)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sequence_handle) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'group_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'group_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'group_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'group_identifier)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'timestamp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'user_handle) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abort_details)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'abort_details)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'abort_details)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'abort_details)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'connection) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SequenceInfoNotification>)))
  "Returns string type for a message object of type '<SequenceInfoNotification>"
  "kortex_driver/SequenceInfoNotification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SequenceInfoNotification)))
  "Returns string type for a message object of type 'SequenceInfoNotification"
  "kortex_driver/SequenceInfoNotification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SequenceInfoNotification>)))
  "Returns md5sum for a message object of type '<SequenceInfoNotification>"
  "cfd9f46ae5ef4976e20a716c9bac5aef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SequenceInfoNotification)))
  "Returns md5sum for a message object of type 'SequenceInfoNotification"
  "cfd9f46ae5ef4976e20a716c9bac5aef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SequenceInfoNotification>)))
  "Returns full string definition for message of type '<SequenceInfoNotification>"
  (cl:format cl:nil "~%uint32 event_identifier~%SequenceHandle sequence_handle~%uint32 task_index~%uint32 group_identifier~%Timestamp timestamp~%UserProfileHandle user_handle~%uint32 abort_details~%Connection connection~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SequenceInfoNotification)))
  "Returns full string definition for message of type 'SequenceInfoNotification"
  (cl:format cl:nil "~%uint32 event_identifier~%SequenceHandle sequence_handle~%uint32 task_index~%uint32 group_identifier~%Timestamp timestamp~%UserProfileHandle user_handle~%uint32 abort_details~%Connection connection~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SequenceInfoNotification>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sequence_handle))
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'timestamp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'user_handle))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'connection))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SequenceInfoNotification>))
  "Converts a ROS message object to a list"
  (cl:list 'SequenceInfoNotification
    (cl:cons ':event_identifier (event_identifier msg))
    (cl:cons ':sequence_handle (sequence_handle msg))
    (cl:cons ':task_index (task_index msg))
    (cl:cons ':group_identifier (group_identifier msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':user_handle (user_handle msg))
    (cl:cons ':abort_details (abort_details msg))
    (cl:cons ':connection (connection msg))
))
