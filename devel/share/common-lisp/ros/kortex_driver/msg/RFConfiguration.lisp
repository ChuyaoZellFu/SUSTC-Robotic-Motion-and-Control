; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude RFConfiguration.msg.html

(cl:defclass <RFConfiguration> (roslisp-msg-protocol:ros-message)
  ((wifi_enable_state
    :reader wifi_enable_state
    :initarg :wifi_enable_state
    :type kortex_driver-msg:WifiEnableState
    :initform (cl:make-instance 'kortex_driver-msg:WifiEnableState))
   (bluetooth_enable_state
    :reader bluetooth_enable_state
    :initarg :bluetooth_enable_state
    :type kortex_driver-msg:BluetoothEnableState
    :initform (cl:make-instance 'kortex_driver-msg:BluetoothEnableState)))
)

(cl:defclass RFConfiguration (<RFConfiguration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RFConfiguration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RFConfiguration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<RFConfiguration> is deprecated: use kortex_driver-msg:RFConfiguration instead.")))

(cl:ensure-generic-function 'wifi_enable_state-val :lambda-list '(m))
(cl:defmethod wifi_enable_state-val ((m <RFConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:wifi_enable_state-val is deprecated.  Use kortex_driver-msg:wifi_enable_state instead.")
  (wifi_enable_state m))

(cl:ensure-generic-function 'bluetooth_enable_state-val :lambda-list '(m))
(cl:defmethod bluetooth_enable_state-val ((m <RFConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:bluetooth_enable_state-val is deprecated.  Use kortex_driver-msg:bluetooth_enable_state instead.")
  (bluetooth_enable_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RFConfiguration>) ostream)
  "Serializes a message object of type '<RFConfiguration>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wifi_enable_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bluetooth_enable_state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RFConfiguration>) istream)
  "Deserializes a message object of type '<RFConfiguration>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wifi_enable_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bluetooth_enable_state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RFConfiguration>)))
  "Returns string type for a message object of type '<RFConfiguration>"
  "kortex_driver/RFConfiguration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RFConfiguration)))
  "Returns string type for a message object of type 'RFConfiguration"
  "kortex_driver/RFConfiguration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RFConfiguration>)))
  "Returns md5sum for a message object of type '<RFConfiguration>"
  "b8bfe540c76a207913f7638bb2dbbae6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RFConfiguration)))
  "Returns md5sum for a message object of type 'RFConfiguration"
  "b8bfe540c76a207913f7638bb2dbbae6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RFConfiguration>)))
  "Returns full string definition for message of type '<RFConfiguration>"
  (cl:format cl:nil "~%WifiEnableState wifi_enable_state~%BluetoothEnableState bluetooth_enable_state~%================================================================================~%MSG: kortex_driver/WifiEnableState~%~%bool enabled~%================================================================================~%MSG: kortex_driver/BluetoothEnableState~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RFConfiguration)))
  "Returns full string definition for message of type 'RFConfiguration"
  (cl:format cl:nil "~%WifiEnableState wifi_enable_state~%BluetoothEnableState bluetooth_enable_state~%================================================================================~%MSG: kortex_driver/WifiEnableState~%~%bool enabled~%================================================================================~%MSG: kortex_driver/BluetoothEnableState~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RFConfiguration>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wifi_enable_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bluetooth_enable_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RFConfiguration>))
  "Converts a ROS message object to a list"
  (cl:list 'RFConfiguration
    (cl:cons ':wifi_enable_state (wifi_enable_state msg))
    (cl:cons ':bluetooth_enable_state (bluetooth_enable_state msg))
))
