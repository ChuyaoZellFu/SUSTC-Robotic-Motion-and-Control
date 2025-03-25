; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ToolConfiguration.msg.html

(cl:defclass <ToolConfiguration> (roslisp-msg-protocol:ros-message)
  ((tool_transform
    :reader tool_transform
    :initarg :tool_transform
    :type kortex_driver-msg:CartesianTransform
    :initform (cl:make-instance 'kortex_driver-msg:CartesianTransform))
   (tool_mass
    :reader tool_mass
    :initarg :tool_mass
    :type cl:float
    :initform 0.0)
   (tool_mass_center
    :reader tool_mass_center
    :initarg :tool_mass_center
    :type kortex_driver-msg:ControlConfig_Position
    :initform (cl:make-instance 'kortex_driver-msg:ControlConfig_Position)))
)

(cl:defclass ToolConfiguration (<ToolConfiguration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ToolConfiguration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ToolConfiguration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ToolConfiguration> is deprecated: use kortex_driver-msg:ToolConfiguration instead.")))

(cl:ensure-generic-function 'tool_transform-val :lambda-list '(m))
(cl:defmethod tool_transform-val ((m <ToolConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_transform-val is deprecated.  Use kortex_driver-msg:tool_transform instead.")
  (tool_transform m))

(cl:ensure-generic-function 'tool_mass-val :lambda-list '(m))
(cl:defmethod tool_mass-val ((m <ToolConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_mass-val is deprecated.  Use kortex_driver-msg:tool_mass instead.")
  (tool_mass m))

(cl:ensure-generic-function 'tool_mass_center-val :lambda-list '(m))
(cl:defmethod tool_mass_center-val ((m <ToolConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tool_mass_center-val is deprecated.  Use kortex_driver-msg:tool_mass_center instead.")
  (tool_mass_center m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ToolConfiguration>) ostream)
  "Serializes a message object of type '<ToolConfiguration>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tool_transform) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tool_mass))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tool_mass_center) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ToolConfiguration>) istream)
  "Deserializes a message object of type '<ToolConfiguration>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tool_transform) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tool_mass) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tool_mass_center) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ToolConfiguration>)))
  "Returns string type for a message object of type '<ToolConfiguration>"
  "kortex_driver/ToolConfiguration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToolConfiguration)))
  "Returns string type for a message object of type 'ToolConfiguration"
  "kortex_driver/ToolConfiguration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ToolConfiguration>)))
  "Returns md5sum for a message object of type '<ToolConfiguration>"
  "cf7e6c29cefe7625fb7412c1a3c76941")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ToolConfiguration)))
  "Returns md5sum for a message object of type 'ToolConfiguration"
  "cf7e6c29cefe7625fb7412c1a3c76941")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ToolConfiguration>)))
  "Returns full string definition for message of type '<ToolConfiguration>"
  (cl:format cl:nil "~%CartesianTransform tool_transform~%float32 tool_mass~%ControlConfig_Position tool_mass_center~%================================================================================~%MSG: kortex_driver/CartesianTransform~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ToolConfiguration)))
  "Returns full string definition for message of type 'ToolConfiguration"
  (cl:format cl:nil "~%CartesianTransform tool_transform~%float32 tool_mass~%ControlConfig_Position tool_mass_center~%================================================================================~%MSG: kortex_driver/CartesianTransform~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ToolConfiguration>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tool_transform))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tool_mass_center))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ToolConfiguration>))
  "Converts a ROS message object to a list"
  (cl:list 'ToolConfiguration
    (cl:cons ':tool_transform (tool_transform msg))
    (cl:cons ':tool_mass (tool_mass msg))
    (cl:cons ':tool_mass_center (tool_mass_center msg))
))
