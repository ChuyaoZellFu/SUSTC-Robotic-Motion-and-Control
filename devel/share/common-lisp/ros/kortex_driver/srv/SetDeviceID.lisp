; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetDeviceID-request.msg.html

(cl:defclass <SetDeviceID-request> (roslisp-msg-protocol:ros-message)
  ((device_id
    :reader device_id
    :initarg :device_id
    :type cl:integer
    :initform 0))
)

(cl:defclass SetDeviceID-request (<SetDeviceID-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDeviceID-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDeviceID-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetDeviceID-request> is deprecated: use kortex_driver-srv:SetDeviceID-request instead.")))

(cl:ensure-generic-function 'device_id-val :lambda-list '(m))
(cl:defmethod device_id-val ((m <SetDeviceID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:device_id-val is deprecated.  Use kortex_driver-srv:device_id instead.")
  (device_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDeviceID-request>) ostream)
  "Serializes a message object of type '<SetDeviceID-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDeviceID-request>) istream)
  "Deserializes a message object of type '<SetDeviceID-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'device_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'device_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'device_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'device_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDeviceID-request>)))
  "Returns string type for a service object of type '<SetDeviceID-request>"
  "kortex_driver/SetDeviceIDRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDeviceID-request)))
  "Returns string type for a service object of type 'SetDeviceID-request"
  "kortex_driver/SetDeviceIDRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDeviceID-request>)))
  "Returns md5sum for a message object of type '<SetDeviceID-request>"
  "983e371f5f8e03bba53608e89ded6b57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDeviceID-request)))
  "Returns md5sum for a message object of type 'SetDeviceID-request"
  "983e371f5f8e03bba53608e89ded6b57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDeviceID-request>)))
  "Returns full string definition for message of type '<SetDeviceID-request>"
  (cl:format cl:nil "uint32 device_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDeviceID-request)))
  "Returns full string definition for message of type 'SetDeviceID-request"
  (cl:format cl:nil "uint32 device_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDeviceID-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDeviceID-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDeviceID-request
    (cl:cons ':device_id (device_id msg))
))
;//! \htmlinclude SetDeviceID-response.msg.html

(cl:defclass <SetDeviceID-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetDeviceID-response (<SetDeviceID-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDeviceID-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDeviceID-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetDeviceID-response> is deprecated: use kortex_driver-srv:SetDeviceID-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDeviceID-response>) ostream)
  "Serializes a message object of type '<SetDeviceID-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDeviceID-response>) istream)
  "Deserializes a message object of type '<SetDeviceID-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDeviceID-response>)))
  "Returns string type for a service object of type '<SetDeviceID-response>"
  "kortex_driver/SetDeviceIDResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDeviceID-response)))
  "Returns string type for a service object of type 'SetDeviceID-response"
  "kortex_driver/SetDeviceIDResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDeviceID-response>)))
  "Returns md5sum for a message object of type '<SetDeviceID-response>"
  "983e371f5f8e03bba53608e89ded6b57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDeviceID-response)))
  "Returns md5sum for a message object of type 'SetDeviceID-response"
  "983e371f5f8e03bba53608e89ded6b57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDeviceID-response>)))
  "Returns full string definition for message of type '<SetDeviceID-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDeviceID-response)))
  "Returns full string definition for message of type 'SetDeviceID-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDeviceID-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDeviceID-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDeviceID-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetDeviceID)))
  'SetDeviceID-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetDeviceID)))
  'SetDeviceID-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDeviceID)))
  "Returns string type for a service object of type '<SetDeviceID>"
  "kortex_driver/SetDeviceID")