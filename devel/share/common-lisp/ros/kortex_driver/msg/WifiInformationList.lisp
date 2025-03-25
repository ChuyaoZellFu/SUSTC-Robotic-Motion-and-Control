; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude WifiInformationList.msg.html

(cl:defclass <WifiInformationList> (roslisp-msg-protocol:ros-message)
  ((wifi_information_list
    :reader wifi_information_list
    :initarg :wifi_information_list
    :type (cl:vector kortex_driver-msg:WifiInformation)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:WifiInformation :initial-element (cl:make-instance 'kortex_driver-msg:WifiInformation))))
)

(cl:defclass WifiInformationList (<WifiInformationList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WifiInformationList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WifiInformationList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<WifiInformationList> is deprecated: use kortex_driver-msg:WifiInformationList instead.")))

(cl:ensure-generic-function 'wifi_information_list-val :lambda-list '(m))
(cl:defmethod wifi_information_list-val ((m <WifiInformationList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:wifi_information_list-val is deprecated.  Use kortex_driver-msg:wifi_information_list instead.")
  (wifi_information_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WifiInformationList>) ostream)
  "Serializes a message object of type '<WifiInformationList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'wifi_information_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'wifi_information_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WifiInformationList>) istream)
  "Deserializes a message object of type '<WifiInformationList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'wifi_information_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'wifi_information_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:WifiInformation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WifiInformationList>)))
  "Returns string type for a message object of type '<WifiInformationList>"
  "kortex_driver/WifiInformationList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WifiInformationList)))
  "Returns string type for a message object of type 'WifiInformationList"
  "kortex_driver/WifiInformationList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WifiInformationList>)))
  "Returns md5sum for a message object of type '<WifiInformationList>"
  "e35665ac97a2d2d722059d5f54d85ef4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WifiInformationList)))
  "Returns md5sum for a message object of type 'WifiInformationList"
  "e35665ac97a2d2d722059d5f54d85ef4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WifiInformationList>)))
  "Returns full string definition for message of type '<WifiInformationList>"
  (cl:format cl:nil "~%WifiInformation[] wifi_information_list~%================================================================================~%MSG: kortex_driver/WifiInformation~%~%Ssid ssid~%uint32 security_type~%uint32 encryption_type~%uint32 signal_quality~%int32 signal_strength~%uint32 frequency~%uint32 channel~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WifiInformationList)))
  "Returns full string definition for message of type 'WifiInformationList"
  (cl:format cl:nil "~%WifiInformation[] wifi_information_list~%================================================================================~%MSG: kortex_driver/WifiInformation~%~%Ssid ssid~%uint32 security_type~%uint32 encryption_type~%uint32 signal_quality~%int32 signal_strength~%uint32 frequency~%uint32 channel~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WifiInformationList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'wifi_information_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WifiInformationList>))
  "Converts a ROS message object to a list"
  (cl:list 'WifiInformationList
    (cl:cons ':wifi_information_list (wifi_information_list msg))
))
