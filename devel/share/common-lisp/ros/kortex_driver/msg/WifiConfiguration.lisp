; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude WifiConfiguration.msg.html

(cl:defclass <WifiConfiguration> (roslisp-msg-protocol:ros-message)
  ((ssid
    :reader ssid
    :initarg :ssid
    :type kortex_driver-msg:Ssid
    :initform (cl:make-instance 'kortex_driver-msg:Ssid))
   (security_key
    :reader security_key
    :initarg :security_key
    :type cl:string
    :initform "")
   (connect_automatically
    :reader connect_automatically
    :initarg :connect_automatically
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass WifiConfiguration (<WifiConfiguration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WifiConfiguration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WifiConfiguration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<WifiConfiguration> is deprecated: use kortex_driver-msg:WifiConfiguration instead.")))

(cl:ensure-generic-function 'ssid-val :lambda-list '(m))
(cl:defmethod ssid-val ((m <WifiConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:ssid-val is deprecated.  Use kortex_driver-msg:ssid instead.")
  (ssid m))

(cl:ensure-generic-function 'security_key-val :lambda-list '(m))
(cl:defmethod security_key-val ((m <WifiConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:security_key-val is deprecated.  Use kortex_driver-msg:security_key instead.")
  (security_key m))

(cl:ensure-generic-function 'connect_automatically-val :lambda-list '(m))
(cl:defmethod connect_automatically-val ((m <WifiConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connect_automatically-val is deprecated.  Use kortex_driver-msg:connect_automatically instead.")
  (connect_automatically m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WifiConfiguration>) ostream)
  "Serializes a message object of type '<WifiConfiguration>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ssid) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'security_key))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'security_key))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'connect_automatically) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WifiConfiguration>) istream)
  "Deserializes a message object of type '<WifiConfiguration>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ssid) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'security_key) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'security_key) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'connect_automatically) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WifiConfiguration>)))
  "Returns string type for a message object of type '<WifiConfiguration>"
  "kortex_driver/WifiConfiguration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WifiConfiguration)))
  "Returns string type for a message object of type 'WifiConfiguration"
  "kortex_driver/WifiConfiguration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WifiConfiguration>)))
  "Returns md5sum for a message object of type '<WifiConfiguration>"
  "4308400b333c62f7c2b092d1ea664706")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WifiConfiguration)))
  "Returns md5sum for a message object of type 'WifiConfiguration"
  "4308400b333c62f7c2b092d1ea664706")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WifiConfiguration>)))
  "Returns full string definition for message of type '<WifiConfiguration>"
  (cl:format cl:nil "~%Ssid ssid~%string security_key~%bool connect_automatically~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WifiConfiguration)))
  "Returns full string definition for message of type 'WifiConfiguration"
  (cl:format cl:nil "~%Ssid ssid~%string security_key~%bool connect_automatically~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WifiConfiguration>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ssid))
     4 (cl:length (cl:slot-value msg 'security_key))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WifiConfiguration>))
  "Converts a ROS message object to a list"
  (cl:list 'WifiConfiguration
    (cl:cons ':ssid (ssid msg))
    (cl:cons ':security_key (security_key msg))
    (cl:cons ':connect_automatically (connect_automatically msg))
))
