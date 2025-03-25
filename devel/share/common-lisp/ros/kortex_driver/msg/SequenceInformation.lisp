; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SequenceInformation.msg.html

(cl:defclass <SequenceInformation> (roslisp-msg-protocol:ros-message)
  ((event_identifier
    :reader event_identifier
    :initarg :event_identifier
    :type cl:integer
    :initform 0)
   (task_index
    :reader task_index
    :initarg :task_index
    :type cl:integer
    :initform 0)
   (task_identifier
    :reader task_identifier
    :initarg :task_identifier
    :type cl:integer
    :initform 0))
)

(cl:defclass SequenceInformation (<SequenceInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SequenceInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SequenceInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SequenceInformation> is deprecated: use kortex_driver-msg:SequenceInformation instead.")))

(cl:ensure-generic-function 'event_identifier-val :lambda-list '(m))
(cl:defmethod event_identifier-val ((m <SequenceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:event_identifier-val is deprecated.  Use kortex_driver-msg:event_identifier instead.")
  (event_identifier m))

(cl:ensure-generic-function 'task_index-val :lambda-list '(m))
(cl:defmethod task_index-val ((m <SequenceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:task_index-val is deprecated.  Use kortex_driver-msg:task_index instead.")
  (task_index m))

(cl:ensure-generic-function 'task_identifier-val :lambda-list '(m))
(cl:defmethod task_identifier-val ((m <SequenceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:task_identifier-val is deprecated.  Use kortex_driver-msg:task_identifier instead.")
  (task_identifier m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SequenceInformation>) ostream)
  "Serializes a message object of type '<SequenceInformation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'task_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'task_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'task_identifier)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SequenceInformation>) istream)
  "Deserializes a message object of type '<SequenceInformation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'task_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'task_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'task_identifier)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SequenceInformation>)))
  "Returns string type for a message object of type '<SequenceInformation>"
  "kortex_driver/SequenceInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SequenceInformation)))
  "Returns string type for a message object of type 'SequenceInformation"
  "kortex_driver/SequenceInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SequenceInformation>)))
  "Returns md5sum for a message object of type '<SequenceInformation>"
  "a7eedb18a251848053b5142e4934ee01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SequenceInformation)))
  "Returns md5sum for a message object of type 'SequenceInformation"
  "a7eedb18a251848053b5142e4934ee01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SequenceInformation>)))
  "Returns full string definition for message of type '<SequenceInformation>"
  (cl:format cl:nil "~%uint32 event_identifier~%uint32 task_index~%uint32 task_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SequenceInformation)))
  "Returns full string definition for message of type 'SequenceInformation"
  (cl:format cl:nil "~%uint32 event_identifier~%uint32 task_index~%uint32 task_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SequenceInformation>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SequenceInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'SequenceInformation
    (cl:cons ':event_identifier (event_identifier msg))
    (cl:cons ':task_index (task_index msg))
    (cl:cons ':task_identifier (task_identifier msg))
))
