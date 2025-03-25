; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude WifiConfigurationList.msg.html

(cl:defclass <WifiConfigurationList> (roslisp-msg-protocol:ros-message)
  ((wifi_configuration_list
    :reader wifi_configuration_list
    :initarg :wifi_configuration_list
    :type (cl:vector kortex_driver-msg:WifiConfiguration)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:WifiConfiguration :initial-element (cl:make-instance 'kortex_driver-msg:WifiConfiguration))))
)

(cl:defclass WifiConfigurationList (<WifiConfigurationList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WifiConfigurationList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WifiConfigurationList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<WifiConfigurationList> is deprecated: use kortex_driver-msg:WifiConfigurationList instead.")))

(cl:ensure-generic-function 'wifi_configuration_list-val :lambda-list '(m))
(cl:defmethod wifi_configuration_list-val ((m <WifiConfigurationList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:wifi_configuration_list-val is deprecated.  Use kortex_driver-msg:wifi_configuration_list instead.")
  (wifi_configuration_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WifiConfigurationList>) ostream)
  "Serializes a message object of type '<WifiConfigurationList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'wifi_configuration_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'wifi_configuration_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WifiConfigurationList>) istream)
  "Deserializes a message object of type '<WifiConfigurationList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'wifi_configuration_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'wifi_configuration_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:WifiConfiguration))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WifiConfigurationList>)))
  "Returns string type for a message object of type '<WifiConfigurationList>"
  "kortex_driver/WifiConfigurationList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WifiConfigurationList)))
  "Returns string type for a message object of type 'WifiConfigurationList"
  "kortex_driver/WifiConfigurationList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WifiConfigurationList>)))
  "Returns md5sum for a message object of type '<WifiConfigurationList>"
  "826e37c04fad51b5554819fbd5e69cc8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WifiConfigurationList)))
  "Returns md5sum for a message object of type 'WifiConfigurationList"
  "826e37c04fad51b5554819fbd5e69cc8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WifiConfigurationList>)))
  "Returns full string definition for message of type '<WifiConfigurationList>"
  (cl:format cl:nil "~%WifiConfiguration[] wifi_configuration_list~%================================================================================~%MSG: kortex_driver/WifiConfiguration~%~%Ssid ssid~%string security_key~%bool connect_automatically~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WifiConfigurationList)))
  "Returns full string definition for message of type 'WifiConfigurationList"
  (cl:format cl:nil "~%WifiConfiguration[] wifi_configuration_list~%================================================================================~%MSG: kortex_driver/WifiConfiguration~%~%Ssid ssid~%string security_key~%bool connect_automatically~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WifiConfigurationList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'wifi_configuration_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WifiConfigurationList>))
  "Converts a ROS message object to a list"
  (cl:list 'WifiConfigurationList
    (cl:cons ':wifi_configuration_list (wifi_configuration_list msg))
))
