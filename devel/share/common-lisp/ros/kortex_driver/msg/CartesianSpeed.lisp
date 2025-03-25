; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CartesianSpeed.msg.html

(cl:defclass <CartesianSpeed> (roslisp-msg-protocol:ros-message)
  ((translation
    :reader translation
    :initarg :translation
    :type cl:float
    :initform 0.0)
   (orientation
    :reader orientation
    :initarg :orientation
    :type cl:float
    :initform 0.0))
)

(cl:defclass CartesianSpeed (<CartesianSpeed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianSpeed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianSpeed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CartesianSpeed> is deprecated: use kortex_driver-msg:CartesianSpeed instead.")))

(cl:ensure-generic-function 'translation-val :lambda-list '(m))
(cl:defmethod translation-val ((m <CartesianSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:translation-val is deprecated.  Use kortex_driver-msg:translation instead.")
  (translation m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <CartesianSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:orientation-val is deprecated.  Use kortex_driver-msg:orientation instead.")
  (orientation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianSpeed>) ostream)
  "Serializes a message object of type '<CartesianSpeed>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'translation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'orientation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianSpeed>) istream)
  "Deserializes a message object of type '<CartesianSpeed>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'translation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'orientation) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianSpeed>)))
  "Returns string type for a message object of type '<CartesianSpeed>"
  "kortex_driver/CartesianSpeed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianSpeed)))
  "Returns string type for a message object of type 'CartesianSpeed"
  "kortex_driver/CartesianSpeed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianSpeed>)))
  "Returns md5sum for a message object of type '<CartesianSpeed>"
  "0fa17c7fbcac20ea2a06fae93c7ef126")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianSpeed)))
  "Returns md5sum for a message object of type 'CartesianSpeed"
  "0fa17c7fbcac20ea2a06fae93c7ef126")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianSpeed>)))
  "Returns full string definition for message of type '<CartesianSpeed>"
  (cl:format cl:nil "~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianSpeed)))
  "Returns full string definition for message of type 'CartesianSpeed"
  (cl:format cl:nil "~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianSpeed>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianSpeed>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianSpeed
    (cl:cons ':translation (translation msg))
    (cl:cons ':orientation (orientation msg))
))
