; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude WifiInformation.msg.html

(cl:defclass <WifiInformation> (roslisp-msg-protocol:ros-message)
  ((ssid
    :reader ssid
    :initarg :ssid
    :type kortex_driver-msg:Ssid
    :initform (cl:make-instance 'kortex_driver-msg:Ssid))
   (security_type
    :reader security_type
    :initarg :security_type
    :type cl:integer
    :initform 0)
   (encryption_type
    :reader encryption_type
    :initarg :encryption_type
    :type cl:integer
    :initform 0)
   (signal_quality
    :reader signal_quality
    :initarg :signal_quality
    :type cl:integer
    :initform 0)
   (signal_strength
    :reader signal_strength
    :initarg :signal_strength
    :type cl:integer
    :initform 0)
   (frequency
    :reader frequency
    :initarg :frequency
    :type cl:integer
    :initform 0)
   (channel
    :reader channel
    :initarg :channel
    :type cl:integer
    :initform 0))
)

(cl:defclass WifiInformation (<WifiInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WifiInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WifiInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<WifiInformation> is deprecated: use kortex_driver-msg:WifiInformation instead.")))

(cl:ensure-generic-function 'ssid-val :lambda-list '(m))
(cl:defmethod ssid-val ((m <WifiInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:ssid-val is deprecated.  Use kortex_driver-msg:ssid instead.")
  (ssid m))

(cl:ensure-generic-function 'security_type-val :lambda-list '(m))
(cl:defmethod security_type-val ((m <WifiInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:security_type-val is deprecated.  Use kortex_driver-msg:security_type instead.")
  (security_type m))

(cl:ensure-generic-function 'encryption_type-val :lambda-list '(m))
(cl:defmethod encryption_type-val ((m <WifiInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:encryption_type-val is deprecated.  Use kortex_driver-msg:encryption_type instead.")
  (encryption_type m))

(cl:ensure-generic-function 'signal_quality-val :lambda-list '(m))
(cl:defmethod signal_quality-val ((m <WifiInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:signal_quality-val is deprecated.  Use kortex_driver-msg:signal_quality instead.")
  (signal_quality m))

(cl:ensure-generic-function 'signal_strength-val :lambda-list '(m))
(cl:defmethod signal_strength-val ((m <WifiInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:signal_strength-val is deprecated.  Use kortex_driver-msg:signal_strength instead.")
  (signal_strength m))

(cl:ensure-generic-function 'frequency-val :lambda-list '(m))
(cl:defmethod frequency-val ((m <WifiInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:frequency-val is deprecated.  Use kortex_driver-msg:frequency instead.")
  (frequency m))

(cl:ensure-generic-function 'channel-val :lambda-list '(m))
(cl:defmethod channel-val ((m <WifiInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:channel-val is deprecated.  Use kortex_driver-msg:channel instead.")
  (channel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WifiInformation>) ostream)
  "Serializes a message object of type '<WifiInformation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ssid) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'security_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'security_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'security_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'security_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'encryption_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'encryption_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'encryption_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'encryption_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'signal_quality)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'signal_quality)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'signal_quality)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'signal_quality)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'signal_strength)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'channel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'channel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'channel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'channel)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WifiInformation>) istream)
  "Deserializes a message object of type '<WifiInformation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ssid) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'security_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'security_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'security_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'security_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'encryption_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'encryption_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'encryption_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'encryption_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'signal_quality)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'signal_quality)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'signal_quality)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'signal_quality)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'signal_strength) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'channel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'channel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'channel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'channel)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WifiInformation>)))
  "Returns string type for a message object of type '<WifiInformation>"
  "kortex_driver/WifiInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WifiInformation)))
  "Returns string type for a message object of type 'WifiInformation"
  "kortex_driver/WifiInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WifiInformation>)))
  "Returns md5sum for a message object of type '<WifiInformation>"
  "6cd167c73dec6e8468316540111c37c6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WifiInformation)))
  "Returns md5sum for a message object of type 'WifiInformation"
  "6cd167c73dec6e8468316540111c37c6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WifiInformation>)))
  "Returns full string definition for message of type '<WifiInformation>"
  (cl:format cl:nil "~%Ssid ssid~%uint32 security_type~%uint32 encryption_type~%uint32 signal_quality~%int32 signal_strength~%uint32 frequency~%uint32 channel~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WifiInformation)))
  "Returns full string definition for message of type 'WifiInformation"
  (cl:format cl:nil "~%Ssid ssid~%uint32 security_type~%uint32 encryption_type~%uint32 signal_quality~%int32 signal_strength~%uint32 frequency~%uint32 channel~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WifiInformation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ssid))
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WifiInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'WifiInformation
    (cl:cons ':ssid (ssid msg))
    (cl:cons ':security_type (security_type msg))
    (cl:cons ':encryption_type (encryption_type msg))
    (cl:cons ':signal_quality (signal_quality msg))
    (cl:cons ':signal_strength (signal_strength msg))
    (cl:cons ':frequency (frequency msg))
    (cl:cons ':channel (channel msg))
))
