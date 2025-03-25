; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetSensorSettings-request.msg.html

(cl:defclass <SetSensorSettings-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SensorSettings
    :initform (cl:make-instance 'kortex_driver-msg:SensorSettings)))
)

(cl:defclass SetSensorSettings-request (<SetSensorSettings-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSensorSettings-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSensorSettings-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetSensorSettings-request> is deprecated: use kortex_driver-srv:SetSensorSettings-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetSensorSettings-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSensorSettings-request>) ostream)
  "Serializes a message object of type '<SetSensorSettings-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSensorSettings-request>) istream)
  "Deserializes a message object of type '<SetSensorSettings-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSensorSettings-request>)))
  "Returns string type for a service object of type '<SetSensorSettings-request>"
  "kortex_driver/SetSensorSettingsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSensorSettings-request)))
  "Returns string type for a service object of type 'SetSensorSettings-request"
  "kortex_driver/SetSensorSettingsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSensorSettings-request>)))
  "Returns md5sum for a message object of type '<SetSensorSettings-request>"
  "30daf04e01a3f6e06e8cd7b777f1164c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSensorSettings-request)))
  "Returns md5sum for a message object of type 'SetSensorSettings-request"
  "30daf04e01a3f6e06e8cd7b777f1164c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSensorSettings-request>)))
  "Returns full string definition for message of type '<SetSensorSettings-request>"
  (cl:format cl:nil "SensorSettings input~%~%================================================================================~%MSG: kortex_driver/SensorSettings~%~%uint32 sensor~%uint32 resolution~%uint32 frame_rate~%uint32 bit_rate~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSensorSettings-request)))
  "Returns full string definition for message of type 'SetSensorSettings-request"
  (cl:format cl:nil "SensorSettings input~%~%================================================================================~%MSG: kortex_driver/SensorSettings~%~%uint32 sensor~%uint32 resolution~%uint32 frame_rate~%uint32 bit_rate~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSensorSettings-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSensorSettings-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSensorSettings-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetSensorSettings-response.msg.html

(cl:defclass <SetSensorSettings-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetSensorSettings-response (<SetSensorSettings-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSensorSettings-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSensorSettings-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetSensorSettings-response> is deprecated: use kortex_driver-srv:SetSensorSettings-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetSensorSettings-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSensorSettings-response>) ostream)
  "Serializes a message object of type '<SetSensorSettings-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSensorSettings-response>) istream)
  "Deserializes a message object of type '<SetSensorSettings-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSensorSettings-response>)))
  "Returns string type for a service object of type '<SetSensorSettings-response>"
  "kortex_driver/SetSensorSettingsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSensorSettings-response)))
  "Returns string type for a service object of type 'SetSensorSettings-response"
  "kortex_driver/SetSensorSettingsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSensorSettings-response>)))
  "Returns md5sum for a message object of type '<SetSensorSettings-response>"
  "30daf04e01a3f6e06e8cd7b777f1164c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSensorSettings-response)))
  "Returns md5sum for a message object of type 'SetSensorSettings-response"
  "30daf04e01a3f6e06e8cd7b777f1164c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSensorSettings-response>)))
  "Returns full string definition for message of type '<SetSensorSettings-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSensorSettings-response)))
  "Returns full string definition for message of type 'SetSensorSettings-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSensorSettings-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSensorSettings-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSensorSettings-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSensorSettings)))
  'SetSensorSettings-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSensorSettings)))
  'SetSensorSettings-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSensorSettings)))
  "Returns string type for a service object of type '<SetSensorSettings>"
  "kortex_driver/SetSensorSettings")