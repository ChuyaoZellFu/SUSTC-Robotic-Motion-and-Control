; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SensorFocusAction.msg.html

(cl:defclass <SensorFocusAction> (roslisp-msg-protocol:ros-message)
  ((sensor
    :reader sensor
    :initarg :sensor
    :type cl:integer
    :initform 0)
   (focus_action
    :reader focus_action
    :initarg :focus_action
    :type cl:integer
    :initform 0)
   (oneof_action_parameters
    :reader oneof_action_parameters
    :initarg :oneof_action_parameters
    :type kortex_driver-msg:SensorFocusAction_action_parameters
    :initform (cl:make-instance 'kortex_driver-msg:SensorFocusAction_action_parameters)))
)

(cl:defclass SensorFocusAction (<SensorFocusAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorFocusAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorFocusAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SensorFocusAction> is deprecated: use kortex_driver-msg:SensorFocusAction instead.")))

(cl:ensure-generic-function 'sensor-val :lambda-list '(m))
(cl:defmethod sensor-val ((m <SensorFocusAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sensor-val is deprecated.  Use kortex_driver-msg:sensor instead.")
  (sensor m))

(cl:ensure-generic-function 'focus_action-val :lambda-list '(m))
(cl:defmethod focus_action-val ((m <SensorFocusAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:focus_action-val is deprecated.  Use kortex_driver-msg:focus_action instead.")
  (focus_action m))

(cl:ensure-generic-function 'oneof_action_parameters-val :lambda-list '(m))
(cl:defmethod oneof_action_parameters-val ((m <SensorFocusAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:oneof_action_parameters-val is deprecated.  Use kortex_driver-msg:oneof_action_parameters instead.")
  (oneof_action_parameters m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorFocusAction>) ostream)
  "Serializes a message object of type '<SensorFocusAction>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'focus_action)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'focus_action)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'focus_action)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'focus_action)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'oneof_action_parameters) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorFocusAction>) istream)
  "Deserializes a message object of type '<SensorFocusAction>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'focus_action)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'focus_action)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'focus_action)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'focus_action)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'oneof_action_parameters) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorFocusAction>)))
  "Returns string type for a message object of type '<SensorFocusAction>"
  "kortex_driver/SensorFocusAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorFocusAction)))
  "Returns string type for a message object of type 'SensorFocusAction"
  "kortex_driver/SensorFocusAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorFocusAction>)))
  "Returns md5sum for a message object of type '<SensorFocusAction>"
  "cae3081e3d5f90fbdd9351620859003c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorFocusAction)))
  "Returns md5sum for a message object of type 'SensorFocusAction"
  "cae3081e3d5f90fbdd9351620859003c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorFocusAction>)))
  "Returns full string definition for message of type '<SensorFocusAction>"
  (cl:format cl:nil "~%uint32 sensor~%uint32 focus_action~%SensorFocusAction_action_parameters oneof_action_parameters~%================================================================================~%MSG: kortex_driver/SensorFocusAction_action_parameters~%~%FocusPoint[] focus_point~%ManualFocus[] manual_focus~%================================================================================~%MSG: kortex_driver/FocusPoint~%~%uint32 x~%uint32 y~%================================================================================~%MSG: kortex_driver/ManualFocus~%~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorFocusAction)))
  "Returns full string definition for message of type 'SensorFocusAction"
  (cl:format cl:nil "~%uint32 sensor~%uint32 focus_action~%SensorFocusAction_action_parameters oneof_action_parameters~%================================================================================~%MSG: kortex_driver/SensorFocusAction_action_parameters~%~%FocusPoint[] focus_point~%ManualFocus[] manual_focus~%================================================================================~%MSG: kortex_driver/FocusPoint~%~%uint32 x~%uint32 y~%================================================================================~%MSG: kortex_driver/ManualFocus~%~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorFocusAction>))
  (cl:+ 0
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'oneof_action_parameters))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorFocusAction>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorFocusAction
    (cl:cons ':sensor (sensor msg))
    (cl:cons ':focus_action (focus_action msg))
    (cl:cons ':oneof_action_parameters (oneof_action_parameters msg))
))
