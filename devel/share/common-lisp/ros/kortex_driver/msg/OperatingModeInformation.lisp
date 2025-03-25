; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude OperatingModeInformation.msg.html

(cl:defclass <OperatingModeInformation> (roslisp-msg-protocol:ros-message)
  ((operating_mode
    :reader operating_mode
    :initarg :operating_mode
    :type cl:integer
    :initform 0)
   (device_handle
    :reader device_handle
    :initarg :device_handle
    :type kortex_driver-msg:DeviceHandle
    :initform (cl:make-instance 'kortex_driver-msg:DeviceHandle)))
)

(cl:defclass OperatingModeInformation (<OperatingModeInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OperatingModeInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OperatingModeInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<OperatingModeInformation> is deprecated: use kortex_driver-msg:OperatingModeInformation instead.")))

(cl:ensure-generic-function 'operating_mode-val :lambda-list '(m))
(cl:defmethod operating_mode-val ((m <OperatingModeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:operating_mode-val is deprecated.  Use kortex_driver-msg:operating_mode instead.")
  (operating_mode m))

(cl:ensure-generic-function 'device_handle-val :lambda-list '(m))
(cl:defmethod device_handle-val ((m <OperatingModeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device_handle-val is deprecated.  Use kortex_driver-msg:device_handle instead.")
  (device_handle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OperatingModeInformation>) ostream)
  "Serializes a message object of type '<OperatingModeInformation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'operating_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'operating_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'operating_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'operating_mode)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'device_handle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OperatingModeInformation>) istream)
  "Deserializes a message object of type '<OperatingModeInformation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'operating_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'operating_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'operating_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'operating_mode)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'device_handle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OperatingModeInformation>)))
  "Returns string type for a message object of type '<OperatingModeInformation>"
  "kortex_driver/OperatingModeInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OperatingModeInformation)))
  "Returns string type for a message object of type 'OperatingModeInformation"
  "kortex_driver/OperatingModeInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OperatingModeInformation>)))
  "Returns md5sum for a message object of type '<OperatingModeInformation>"
  "c45e380eac0023b4784c990d000302a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OperatingModeInformation)))
  "Returns md5sum for a message object of type 'OperatingModeInformation"
  "c45e380eac0023b4784c990d000302a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OperatingModeInformation>)))
  "Returns full string definition for message of type '<OperatingModeInformation>"
  (cl:format cl:nil "~%uint32 operating_mode~%DeviceHandle device_handle~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OperatingModeInformation)))
  "Returns full string definition for message of type 'OperatingModeInformation"
  (cl:format cl:nil "~%uint32 operating_mode~%DeviceHandle device_handle~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OperatingModeInformation>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'device_handle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OperatingModeInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'OperatingModeInformation
    (cl:cons ':operating_mode (operating_mode msg))
    (cl:cons ':device_handle (device_handle msg))
))
