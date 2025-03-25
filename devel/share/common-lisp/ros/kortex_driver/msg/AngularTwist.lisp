; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude AngularTwist.msg.html

(cl:defclass <AngularTwist> (roslisp-msg-protocol:ros-message)
  ((angular
    :reader angular
    :initarg :angular
    :type cl:float
    :initform 0.0))
)

(cl:defclass AngularTwist (<AngularTwist>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AngularTwist>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AngularTwist)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<AngularTwist> is deprecated: use kortex_driver-msg:AngularTwist instead.")))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <AngularTwist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:angular-val is deprecated.  Use kortex_driver-msg:angular instead.")
  (angular m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AngularTwist>) ostream)
  "Serializes a message object of type '<AngularTwist>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AngularTwist>) istream)
  "Deserializes a message object of type '<AngularTwist>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AngularTwist>)))
  "Returns string type for a message object of type '<AngularTwist>"
  "kortex_driver/AngularTwist")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AngularTwist)))
  "Returns string type for a message object of type 'AngularTwist"
  "kortex_driver/AngularTwist")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AngularTwist>)))
  "Returns md5sum for a message object of type '<AngularTwist>"
  "960aa4f2aaeac14706e7fa1a905e8a00")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AngularTwist)))
  "Returns md5sum for a message object of type 'AngularTwist"
  "960aa4f2aaeac14706e7fa1a905e8a00")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AngularTwist>)))
  "Returns full string definition for message of type '<AngularTwist>"
  (cl:format cl:nil "~%float32 angular~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AngularTwist)))
  "Returns full string definition for message of type 'AngularTwist"
  (cl:format cl:nil "~%float32 angular~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AngularTwist>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AngularTwist>))
  "Converts a ROS message object to a list"
  (cl:list 'AngularTwist
    (cl:cons ':angular (angular msg))
))
