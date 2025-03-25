; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetIntrinsicParameters-request.msg.html

(cl:defclass <GetIntrinsicParameters-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SensorIdentifier
    :initform (cl:make-instance 'kortex_driver-msg:SensorIdentifier)))
)

(cl:defclass GetIntrinsicParameters-request (<GetIntrinsicParameters-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIntrinsicParameters-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIntrinsicParameters-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetIntrinsicParameters-request> is deprecated: use kortex_driver-srv:GetIntrinsicParameters-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetIntrinsicParameters-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIntrinsicParameters-request>) ostream)
  "Serializes a message object of type '<GetIntrinsicParameters-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIntrinsicParameters-request>) istream)
  "Deserializes a message object of type '<GetIntrinsicParameters-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIntrinsicParameters-request>)))
  "Returns string type for a service object of type '<GetIntrinsicParameters-request>"
  "kortex_driver/GetIntrinsicParametersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntrinsicParameters-request)))
  "Returns string type for a service object of type 'GetIntrinsicParameters-request"
  "kortex_driver/GetIntrinsicParametersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIntrinsicParameters-request>)))
  "Returns md5sum for a message object of type '<GetIntrinsicParameters-request>"
  "b088fe2711ccce33e17ff2a6a6244b06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIntrinsicParameters-request)))
  "Returns md5sum for a message object of type 'GetIntrinsicParameters-request"
  "b088fe2711ccce33e17ff2a6a6244b06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIntrinsicParameters-request>)))
  "Returns full string definition for message of type '<GetIntrinsicParameters-request>"
  (cl:format cl:nil "SensorIdentifier input~%~%================================================================================~%MSG: kortex_driver/SensorIdentifier~%~%uint32 sensor~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIntrinsicParameters-request)))
  "Returns full string definition for message of type 'GetIntrinsicParameters-request"
  (cl:format cl:nil "SensorIdentifier input~%~%================================================================================~%MSG: kortex_driver/SensorIdentifier~%~%uint32 sensor~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIntrinsicParameters-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIntrinsicParameters-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIntrinsicParameters-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetIntrinsicParameters-response.msg.html

(cl:defclass <GetIntrinsicParameters-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:IntrinsicParameters
    :initform (cl:make-instance 'kortex_driver-msg:IntrinsicParameters)))
)

(cl:defclass GetIntrinsicParameters-response (<GetIntrinsicParameters-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIntrinsicParameters-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIntrinsicParameters-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetIntrinsicParameters-response> is deprecated: use kortex_driver-srv:GetIntrinsicParameters-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetIntrinsicParameters-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIntrinsicParameters-response>) ostream)
  "Serializes a message object of type '<GetIntrinsicParameters-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIntrinsicParameters-response>) istream)
  "Deserializes a message object of type '<GetIntrinsicParameters-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIntrinsicParameters-response>)))
  "Returns string type for a service object of type '<GetIntrinsicParameters-response>"
  "kortex_driver/GetIntrinsicParametersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntrinsicParameters-response)))
  "Returns string type for a service object of type 'GetIntrinsicParameters-response"
  "kortex_driver/GetIntrinsicParametersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIntrinsicParameters-response>)))
  "Returns md5sum for a message object of type '<GetIntrinsicParameters-response>"
  "b088fe2711ccce33e17ff2a6a6244b06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIntrinsicParameters-response)))
  "Returns md5sum for a message object of type 'GetIntrinsicParameters-response"
  "b088fe2711ccce33e17ff2a6a6244b06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIntrinsicParameters-response>)))
  "Returns full string definition for message of type '<GetIntrinsicParameters-response>"
  (cl:format cl:nil "IntrinsicParameters output~%~%================================================================================~%MSG: kortex_driver/IntrinsicParameters~%~%uint32 sensor~%uint32 resolution~%float32 principal_point_x~%float32 principal_point_y~%float32 focal_length_x~%float32 focal_length_y~%DistortionCoefficients distortion_coeffs~%================================================================================~%MSG: kortex_driver/DistortionCoefficients~%~%float32 k1~%float32 k2~%float32 k3~%float32 p1~%float32 p2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIntrinsicParameters-response)))
  "Returns full string definition for message of type 'GetIntrinsicParameters-response"
  (cl:format cl:nil "IntrinsicParameters output~%~%================================================================================~%MSG: kortex_driver/IntrinsicParameters~%~%uint32 sensor~%uint32 resolution~%float32 principal_point_x~%float32 principal_point_y~%float32 focal_length_x~%float32 focal_length_y~%DistortionCoefficients distortion_coeffs~%================================================================================~%MSG: kortex_driver/DistortionCoefficients~%~%float32 k1~%float32 k2~%float32 k3~%float32 p1~%float32 p2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIntrinsicParameters-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIntrinsicParameters-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIntrinsicParameters-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetIntrinsicParameters)))
  'GetIntrinsicParameters-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetIntrinsicParameters)))
  'GetIntrinsicParameters-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntrinsicParameters)))
  "Returns string type for a service object of type '<GetIntrinsicParameters>"
  "kortex_driver/GetIntrinsicParameters")