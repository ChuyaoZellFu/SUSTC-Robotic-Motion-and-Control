; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SequenceTasksRange.msg.html

(cl:defclass <SequenceTasksRange> (roslisp-msg-protocol:ros-message)
  ((first_task_index
    :reader first_task_index
    :initarg :first_task_index
    :type cl:integer
    :initform 0)
   (second_task_index
    :reader second_task_index
    :initarg :second_task_index
    :type cl:integer
    :initform 0))
)

(cl:defclass SequenceTasksRange (<SequenceTasksRange>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SequenceTasksRange>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SequenceTasksRange)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SequenceTasksRange> is deprecated: use kortex_driver-msg:SequenceTasksRange instead.")))

(cl:ensure-generic-function 'first_task_index-val :lambda-list '(m))
(cl:defmethod first_task_index-val ((m <SequenceTasksRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:first_task_index-val is deprecated.  Use kortex_driver-msg:first_task_index instead.")
  (first_task_index m))

(cl:ensure-generic-function 'second_task_index-val :lambda-list '(m))
(cl:defmethod second_task_index-val ((m <SequenceTasksRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:second_task_index-val is deprecated.  Use kortex_driver-msg:second_task_index instead.")
  (second_task_index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SequenceTasksRange>) ostream)
  "Serializes a message object of type '<SequenceTasksRange>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'first_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'first_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'first_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'first_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'second_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'second_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'second_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'second_task_index)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SequenceTasksRange>) istream)
  "Deserializes a message object of type '<SequenceTasksRange>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'first_task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'first_task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'first_task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'first_task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'second_task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'second_task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'second_task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'second_task_index)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SequenceTasksRange>)))
  "Returns string type for a message object of type '<SequenceTasksRange>"
  "kortex_driver/SequenceTasksRange")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SequenceTasksRange)))
  "Returns string type for a message object of type 'SequenceTasksRange"
  "kortex_driver/SequenceTasksRange")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SequenceTasksRange>)))
  "Returns md5sum for a message object of type '<SequenceTasksRange>"
  "f9dba2f68de4791eab54435a0c4b312c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SequenceTasksRange)))
  "Returns md5sum for a message object of type 'SequenceTasksRange"
  "f9dba2f68de4791eab54435a0c4b312c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SequenceTasksRange>)))
  "Returns full string definition for message of type '<SequenceTasksRange>"
  (cl:format cl:nil "~%uint32 first_task_index~%uint32 second_task_index~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SequenceTasksRange)))
  "Returns full string definition for message of type 'SequenceTasksRange"
  (cl:format cl:nil "~%uint32 first_task_index~%uint32 second_task_index~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SequenceTasksRange>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SequenceTasksRange>))
  "Converts a ROS message object to a list"
  (cl:list 'SequenceTasksRange
    (cl:cons ':first_task_index (first_task_index msg))
    (cl:cons ':second_task_index (second_task_index msg))
))
