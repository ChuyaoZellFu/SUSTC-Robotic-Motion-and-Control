; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerConfigurationMode.msg.html

(cl:defclass <ControllerConfigurationMode> (roslisp-msg-protocol:ros-message)
  ((enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ControllerConfigurationMode (<ControllerConfigurationMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerConfigurationMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerConfigurationMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerConfigurationMode> is deprecated: use kortex_driver-msg:ControllerConfigurationMode instead.")))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <ControllerConfigurationMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:enable-val is deprecated.  Use kortex_driver-msg:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerConfigurationMode>) ostream)
  "Serializes a message object of type '<ControllerConfigurationMode>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerConfigurationMode>) istream)
  "Deserializes a message object of type '<ControllerConfigurationMode>"
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerConfigurationMode>)))
  "Returns string type for a message object of type '<ControllerConfigurationMode>"
  "kortex_driver/ControllerConfigurationMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerConfigurationMode)))
  "Returns string type for a message object of type 'ControllerConfigurationMode"
  "kortex_driver/ControllerConfigurationMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerConfigurationMode>)))
  "Returns md5sum for a message object of type '<ControllerConfigurationMode>"
  "8c1211af706069c994c06e00eb59ac9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerConfigurationMode)))
  "Returns md5sum for a message object of type 'ControllerConfigurationMode"
  "8c1211af706069c994c06e00eb59ac9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerConfigurationMode>)))
  "Returns full string definition for message of type '<ControllerConfigurationMode>"
  (cl:format cl:nil "~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerConfigurationMode)))
  "Returns full string definition for message of type 'ControllerConfigurationMode"
  (cl:format cl:nil "~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerConfigurationMode>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerConfigurationMode>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerConfigurationMode
    (cl:cons ':enable (enable msg))
))
