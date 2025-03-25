; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Finger.msg.html

(cl:defclass <Finger> (roslisp-msg-protocol:ros-message)
  ((finger_identifier
    :reader finger_identifier
    :initarg :finger_identifier
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass Finger (<Finger>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Finger>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Finger)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Finger> is deprecated: use kortex_driver-msg:Finger instead.")))

(cl:ensure-generic-function 'finger_identifier-val :lambda-list '(m))
(cl:defmethod finger_identifier-val ((m <Finger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:finger_identifier-val is deprecated.  Use kortex_driver-msg:finger_identifier instead.")
  (finger_identifier m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Finger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:value-val is deprecated.  Use kortex_driver-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Finger>) ostream)
  "Serializes a message object of type '<Finger>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'finger_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'finger_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'finger_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'finger_identifier)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Finger>) istream)
  "Deserializes a message object of type '<Finger>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'finger_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'finger_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'finger_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'finger_identifier)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Finger>)))
  "Returns string type for a message object of type '<Finger>"
  "kortex_driver/Finger")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Finger)))
  "Returns string type for a message object of type 'Finger"
  "kortex_driver/Finger")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Finger>)))
  "Returns md5sum for a message object of type '<Finger>"
  "5bf3287ad42d5a92136c7d9cf8d0eada")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Finger)))
  "Returns md5sum for a message object of type 'Finger"
  "5bf3287ad42d5a92136c7d9cf8d0eada")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Finger>)))
  "Returns full string definition for message of type '<Finger>"
  (cl:format cl:nil "~%uint32 finger_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Finger)))
  "Returns full string definition for message of type 'Finger"
  (cl:format cl:nil "~%uint32 finger_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Finger>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Finger>))
  "Converts a ROS message object to a list"
  (cl:list 'Finger
    (cl:cons ':finger_identifier (finger_identifier msg))
    (cl:cons ':value (value msg))
))
