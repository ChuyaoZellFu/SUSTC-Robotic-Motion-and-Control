; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Snapshot.msg.html

(cl:defclass <Snapshot> (roslisp-msg-protocol:ros-message)
  ((snapshot_type
    :reader snapshot_type
    :initarg :snapshot_type
    :type cl:integer
    :initform 0))
)

(cl:defclass Snapshot (<Snapshot>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Snapshot>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Snapshot)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Snapshot> is deprecated: use kortex_driver-msg:Snapshot instead.")))

(cl:ensure-generic-function 'snapshot_type-val :lambda-list '(m))
(cl:defmethod snapshot_type-val ((m <Snapshot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:snapshot_type-val is deprecated.  Use kortex_driver-msg:snapshot_type instead.")
  (snapshot_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Snapshot>) ostream)
  "Serializes a message object of type '<Snapshot>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'snapshot_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'snapshot_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'snapshot_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'snapshot_type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Snapshot>) istream)
  "Deserializes a message object of type '<Snapshot>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'snapshot_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'snapshot_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'snapshot_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'snapshot_type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Snapshot>)))
  "Returns string type for a message object of type '<Snapshot>"
  "kortex_driver/Snapshot")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Snapshot)))
  "Returns string type for a message object of type 'Snapshot"
  "kortex_driver/Snapshot")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Snapshot>)))
  "Returns md5sum for a message object of type '<Snapshot>"
  "d1bcc273f3d53a1ef917f40e2afafc1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Snapshot)))
  "Returns md5sum for a message object of type 'Snapshot"
  "d1bcc273f3d53a1ef917f40e2afafc1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Snapshot>)))
  "Returns full string definition for message of type '<Snapshot>"
  (cl:format cl:nil "~%uint32 snapshot_type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Snapshot)))
  "Returns full string definition for message of type 'Snapshot"
  (cl:format cl:nil "~%uint32 snapshot_type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Snapshot>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Snapshot>))
  "Converts a ROS message object to a list"
  (cl:list 'Snapshot
    (cl:cons ':snapshot_type (snapshot_type msg))
))
