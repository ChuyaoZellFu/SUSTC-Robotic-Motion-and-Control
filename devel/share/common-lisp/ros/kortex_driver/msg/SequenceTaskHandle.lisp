; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SequenceTaskHandle.msg.html

(cl:defclass <SequenceTaskHandle> (roslisp-msg-protocol:ros-message)
  ((sequence_handle
    :reader sequence_handle
    :initarg :sequence_handle
    :type kortex_driver-msg:SequenceHandle
    :initform (cl:make-instance 'kortex_driver-msg:SequenceHandle))
   (task_index
    :reader task_index
    :initarg :task_index
    :type cl:integer
    :initform 0))
)

(cl:defclass SequenceTaskHandle (<SequenceTaskHandle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SequenceTaskHandle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SequenceTaskHandle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SequenceTaskHandle> is deprecated: use kortex_driver-msg:SequenceTaskHandle instead.")))

(cl:ensure-generic-function 'sequence_handle-val :lambda-list '(m))
(cl:defmethod sequence_handle-val ((m <SequenceTaskHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sequence_handle-val is deprecated.  Use kortex_driver-msg:sequence_handle instead.")
  (sequence_handle m))

(cl:ensure-generic-function 'task_index-val :lambda-list '(m))
(cl:defmethod task_index-val ((m <SequenceTaskHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:task_index-val is deprecated.  Use kortex_driver-msg:task_index instead.")
  (task_index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SequenceTaskHandle>) ostream)
  "Serializes a message object of type '<SequenceTaskHandle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sequence_handle) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'task_index)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SequenceTaskHandle>) istream)
  "Deserializes a message object of type '<SequenceTaskHandle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sequence_handle) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'task_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'task_index)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SequenceTaskHandle>)))
  "Returns string type for a message object of type '<SequenceTaskHandle>"
  "kortex_driver/SequenceTaskHandle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SequenceTaskHandle)))
  "Returns string type for a message object of type 'SequenceTaskHandle"
  "kortex_driver/SequenceTaskHandle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SequenceTaskHandle>)))
  "Returns md5sum for a message object of type '<SequenceTaskHandle>"
  "d86cef527dffeac4930486133afaaef1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SequenceTaskHandle)))
  "Returns md5sum for a message object of type 'SequenceTaskHandle"
  "d86cef527dffeac4930486133afaaef1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SequenceTaskHandle>)))
  "Returns full string definition for message of type '<SequenceTaskHandle>"
  (cl:format cl:nil "~%SequenceHandle sequence_handle~%uint32 task_index~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SequenceTaskHandle)))
  "Returns full string definition for message of type 'SequenceTaskHandle"
  (cl:format cl:nil "~%SequenceHandle sequence_handle~%uint32 task_index~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SequenceTaskHandle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sequence_handle))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SequenceTaskHandle>))
  "Converts a ROS message object to a list"
  (cl:list 'SequenceTaskHandle
    (cl:cons ':sequence_handle (sequence_handle msg))
    (cl:cons ':task_index (task_index msg))
))
