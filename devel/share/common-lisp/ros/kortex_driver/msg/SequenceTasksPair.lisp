; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SequenceTasksPair.msg.html

(cl:defclass <SequenceTasksPair> (roslisp-msg-protocol:ros-message)
  ((sequence_handle
    :reader sequence_handle
    :initarg :sequence_handle
    :type kortex_driver-msg:SequenceHandle
    :initform (cl:make-instance 'kortex_driver-msg:SequenceHandle))
   (first_task_index
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

(cl:defclass SequenceTasksPair (<SequenceTasksPair>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SequenceTasksPair>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SequenceTasksPair)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SequenceTasksPair> is deprecated: use kortex_driver-msg:SequenceTasksPair instead.")))

(cl:ensure-generic-function 'sequence_handle-val :lambda-list '(m))
(cl:defmethod sequence_handle-val ((m <SequenceTasksPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sequence_handle-val is deprecated.  Use kortex_driver-msg:sequence_handle instead.")
  (sequence_handle m))

(cl:ensure-generic-function 'first_task_index-val :lambda-list '(m))
(cl:defmethod first_task_index-val ((m <SequenceTasksPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:first_task_index-val is deprecated.  Use kortex_driver-msg:first_task_index instead.")
  (first_task_index m))

(cl:ensure-generic-function 'second_task_index-val :lambda-list '(m))
(cl:defmethod second_task_index-val ((m <SequenceTasksPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:second_task_index-val is deprecated.  Use kortex_driver-msg:second_task_index instead.")
  (second_task_index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SequenceTasksPair>) ostream)
  "Serializes a message object of type '<SequenceTasksPair>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sequence_handle) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'first_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'first_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'first_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'first_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'second_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'second_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'second_task_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'second_task_index)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SequenceTasksPair>) istream)
  "Deserializes a message object of type '<SequenceTasksPair>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sequence_handle) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SequenceTasksPair>)))
  "Returns string type for a message object of type '<SequenceTasksPair>"
  "kortex_driver/SequenceTasksPair")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SequenceTasksPair)))
  "Returns string type for a message object of type 'SequenceTasksPair"
  "kortex_driver/SequenceTasksPair")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SequenceTasksPair>)))
  "Returns md5sum for a message object of type '<SequenceTasksPair>"
  "1e54820745915e6c8b6b14cf1311455d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SequenceTasksPair)))
  "Returns md5sum for a message object of type 'SequenceTasksPair"
  "1e54820745915e6c8b6b14cf1311455d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SequenceTasksPair>)))
  "Returns full string definition for message of type '<SequenceTasksPair>"
  (cl:format cl:nil "~%SequenceHandle sequence_handle~%uint32 first_task_index~%uint32 second_task_index~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SequenceTasksPair)))
  "Returns full string definition for message of type 'SequenceTasksPair"
  (cl:format cl:nil "~%SequenceHandle sequence_handle~%uint32 first_task_index~%uint32 second_task_index~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SequenceTasksPair>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sequence_handle))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SequenceTasksPair>))
  "Converts a ROS message object to a list"
  (cl:list 'SequenceTasksPair
    (cl:cons ':sequence_handle (sequence_handle msg))
    (cl:cons ':first_task_index (first_task_index msg))
    (cl:cons ':second_task_index (second_task_index msg))
))
