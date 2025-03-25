; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetSensorSettings-request.msg.html

(cl:defclass <GetSensorSettings-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SensorIdentifier
    :initform (cl:make-instance 'kortex_driver-msg:SensorIdentifier)))
)

(cl:defclass GetSensorSettings-request (<GetSensorSettings-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSensorSettings-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSensorSettings-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSensorSettings-request> is deprecated: use kortex_driver-srv:GetSensorSettings-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetSensorSettings-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSensorSettings-request>) ostream)
  "Serializes a message object of type '<GetSensorSettings-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSensorSettings-request>) istream)
  "Deserializes a message object of type '<GetSensorSettings-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSensorSettings-request>)))
  "Returns string type for a service object of type '<GetSensorSettings-request>"
  "kortex_driver/GetSensorSettingsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSensorSettings-request)))
  "Returns string type for a service object of type 'GetSensorSettings-request"
  "kortex_driver/GetSensorSettingsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSensorSettings-request>)))
  "Returns md5sum for a message object of type '<GetSensorSettings-request>"
  "16278c9fd5ce95a2e5bda4211043968e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSensorSettings-request)))
  "Returns md5sum for a message object of type 'GetSensorSettings-request"
  "16278c9fd5ce95a2e5bda4211043968e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSensorSettings-request>)))
  "Returns full string definition for message of type '<GetSensorSettings-request>"
  (cl:format cl:nil "SensorIdentifier input~%~%================================================================================~%MSG: kortex_driver/SensorIdentifier~%~%uint32 sensor~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSensorSettings-request)))
  "Returns full string definition for message of type 'GetSensorSettings-request"
  (cl:format cl:nil "SensorIdentifier input~%~%================================================================================~%MSG: kortex_driver/SensorIdentifier~%~%uint32 sensor~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSensorSettings-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSensorSettings-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSensorSettings-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetSensorSettings-response.msg.html

(cl:defclass <GetSensorSettings-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:SensorSettings
    :initform (cl:make-instance 'kortex_driver-msg:SensorSettings)))
)

(cl:defclass GetSensorSettings-response (<GetSensorSettings-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSensorSettings-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSensorSettings-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSensorSettings-response> is deprecated: use kortex_driver-srv:GetSensorSettings-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetSensorSettings-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSensorSettings-response>) ostream)
  "Serializes a message object of type '<GetSensorSettings-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSensorSettings-response>) istream)
  "Deserializes a message object of type '<GetSensorSettings-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSensorSettings-response>)))
  "Returns string type for a service object of type '<GetSensorSettings-response>"
  "kortex_driver/GetSensorSettingsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSensorSettings-response)))
  "Returns string type for a service object of type 'GetSensorSettings-response"
  "kortex_driver/GetSensorSettingsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSensorSettings-response>)))
  "Returns md5sum for a message object of type '<GetSensorSettings-response>"
  "16278c9fd5ce95a2e5bda4211043968e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSensorSettings-response)))
  "Returns md5sum for a message object of type 'GetSensorSettings-response"
  "16278c9fd5ce95a2e5bda4211043968e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSensorSettings-response>)))
  "Returns full string definition for message of type '<GetSensorSettings-response>"
  (cl:format cl:nil "SensorSettings output~%~%================================================================================~%MSG: kortex_driver/SensorSettings~%~%uint32 sensor~%uint32 resolution~%uint32 frame_rate~%uint32 bit_rate~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSensorSettings-response)))
  "Returns full string definition for message of type 'GetSensorSettings-response"
  (cl:format cl:nil "SensorSettings output~%~%================================================================================~%MSG: kortex_driver/SensorSettings~%~%uint32 sensor~%uint32 resolution~%uint32 frame_rate~%uint32 bit_rate~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSensorSettings-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSensorSettings-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSensorSettings-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSensorSettings)))
  'GetSensorSettings-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSensorSettings)))
  'GetSensorSettings-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSensorSettings)))
  "Returns string type for a service object of type '<GetSensorSettings>"
  "kortex_driver/GetSensorSettings")