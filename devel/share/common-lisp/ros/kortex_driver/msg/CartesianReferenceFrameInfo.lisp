; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CartesianReferenceFrameInfo.msg.html

(cl:defclass <CartesianReferenceFrameInfo> (roslisp-msg-protocol:ros-message)
  ((reference_frame
    :reader reference_frame
    :initarg :reference_frame
    :type cl:integer
    :initform 0))
)

(cl:defclass CartesianReferenceFrameInfo (<CartesianReferenceFrameInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianReferenceFrameInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianReferenceFrameInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CartesianReferenceFrameInfo> is deprecated: use kortex_driver-msg:CartesianReferenceFrameInfo instead.")))

(cl:ensure-generic-function 'reference_frame-val :lambda-list '(m))
(cl:defmethod reference_frame-val ((m <CartesianReferenceFrameInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:reference_frame-val is deprecated.  Use kortex_driver-msg:reference_frame instead.")
  (reference_frame m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianReferenceFrameInfo>) ostream)
  "Serializes a message object of type '<CartesianReferenceFrameInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reference_frame)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianReferenceFrameInfo>) istream)
  "Deserializes a message object of type '<CartesianReferenceFrameInfo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianReferenceFrameInfo>)))
  "Returns string type for a message object of type '<CartesianReferenceFrameInfo>"
  "kortex_driver/CartesianReferenceFrameInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianReferenceFrameInfo)))
  "Returns string type for a message object of type 'CartesianReferenceFrameInfo"
  "kortex_driver/CartesianReferenceFrameInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianReferenceFrameInfo>)))
  "Returns md5sum for a message object of type '<CartesianReferenceFrameInfo>"
  "c6e1a124b6ab31faa150246edd7907de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianReferenceFrameInfo)))
  "Returns md5sum for a message object of type 'CartesianReferenceFrameInfo"
  "c6e1a124b6ab31faa150246edd7907de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianReferenceFrameInfo>)))
  "Returns full string definition for message of type '<CartesianReferenceFrameInfo>"
  (cl:format cl:nil "~%uint32 reference_frame~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianReferenceFrameInfo)))
  "Returns full string definition for message of type 'CartesianReferenceFrameInfo"
  (cl:format cl:nil "~%uint32 reference_frame~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianReferenceFrameInfo>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianReferenceFrameInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianReferenceFrameInfo
    (cl:cons ':reference_frame (reference_frame msg))
))
