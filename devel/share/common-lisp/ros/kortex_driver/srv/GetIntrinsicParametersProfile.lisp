; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetIntrinsicParametersProfile-request.msg.html

(cl:defclass <GetIntrinsicParametersProfile-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:IntrinsicProfileIdentifier
    :initform (cl:make-instance 'kortex_driver-msg:IntrinsicProfileIdentifier)))
)

(cl:defclass GetIntrinsicParametersProfile-request (<GetIntrinsicParametersProfile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIntrinsicParametersProfile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIntrinsicParametersProfile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetIntrinsicParametersProfile-request> is deprecated: use kortex_driver-srv:GetIntrinsicParametersProfile-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetIntrinsicParametersProfile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIntrinsicParametersProfile-request>) ostream)
  "Serializes a message object of type '<GetIntrinsicParametersProfile-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIntrinsicParametersProfile-request>) istream)
  "Deserializes a message object of type '<GetIntrinsicParametersProfile-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIntrinsicParametersProfile-request>)))
  "Returns string type for a service object of type '<GetIntrinsicParametersProfile-request>"
  "kortex_driver/GetIntrinsicParametersProfileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntrinsicParametersProfile-request)))
  "Returns string type for a service object of type 'GetIntrinsicParametersProfile-request"
  "kortex_driver/GetIntrinsicParametersProfileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIntrinsicParametersProfile-request>)))
  "Returns md5sum for a message object of type '<GetIntrinsicParametersProfile-request>"
  "9b534af230711bb30b60153a5b06f949")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIntrinsicParametersProfile-request)))
  "Returns md5sum for a message object of type 'GetIntrinsicParametersProfile-request"
  "9b534af230711bb30b60153a5b06f949")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIntrinsicParametersProfile-request>)))
  "Returns full string definition for message of type '<GetIntrinsicParametersProfile-request>"
  (cl:format cl:nil "IntrinsicProfileIdentifier input~%~%================================================================================~%MSG: kortex_driver/IntrinsicProfileIdentifier~%~%uint32 sensor~%uint32 resolution~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIntrinsicParametersProfile-request)))
  "Returns full string definition for message of type 'GetIntrinsicParametersProfile-request"
  (cl:format cl:nil "IntrinsicProfileIdentifier input~%~%================================================================================~%MSG: kortex_driver/IntrinsicProfileIdentifier~%~%uint32 sensor~%uint32 resolution~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIntrinsicParametersProfile-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIntrinsicParametersProfile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIntrinsicParametersProfile-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetIntrinsicParametersProfile-response.msg.html

(cl:defclass <GetIntrinsicParametersProfile-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:IntrinsicParameters
    :initform (cl:make-instance 'kortex_driver-msg:IntrinsicParameters)))
)

(cl:defclass GetIntrinsicParametersProfile-response (<GetIntrinsicParametersProfile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIntrinsicParametersProfile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIntrinsicParametersProfile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetIntrinsicParametersProfile-response> is deprecated: use kortex_driver-srv:GetIntrinsicParametersProfile-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetIntrinsicParametersProfile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIntrinsicParametersProfile-response>) ostream)
  "Serializes a message object of type '<GetIntrinsicParametersProfile-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIntrinsicParametersProfile-response>) istream)
  "Deserializes a message object of type '<GetIntrinsicParametersProfile-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIntrinsicParametersProfile-response>)))
  "Returns string type for a service object of type '<GetIntrinsicParametersProfile-response>"
  "kortex_driver/GetIntrinsicParametersProfileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntrinsicParametersProfile-response)))
  "Returns string type for a service object of type 'GetIntrinsicParametersProfile-response"
  "kortex_driver/GetIntrinsicParametersProfileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIntrinsicParametersProfile-response>)))
  "Returns md5sum for a message object of type '<GetIntrinsicParametersProfile-response>"
  "9b534af230711bb30b60153a5b06f949")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIntrinsicParametersProfile-response)))
  "Returns md5sum for a message object of type 'GetIntrinsicParametersProfile-response"
  "9b534af230711bb30b60153a5b06f949")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIntrinsicParametersProfile-response>)))
  "Returns full string definition for message of type '<GetIntrinsicParametersProfile-response>"
  (cl:format cl:nil "IntrinsicParameters output~%~%================================================================================~%MSG: kortex_driver/IntrinsicParameters~%~%uint32 sensor~%uint32 resolution~%float32 principal_point_x~%float32 principal_point_y~%float32 focal_length_x~%float32 focal_length_y~%DistortionCoefficients distortion_coeffs~%================================================================================~%MSG: kortex_driver/DistortionCoefficients~%~%float32 k1~%float32 k2~%float32 k3~%float32 p1~%float32 p2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIntrinsicParametersProfile-response)))
  "Returns full string definition for message of type 'GetIntrinsicParametersProfile-response"
  (cl:format cl:nil "IntrinsicParameters output~%~%================================================================================~%MSG: kortex_driver/IntrinsicParameters~%~%uint32 sensor~%uint32 resolution~%float32 principal_point_x~%float32 principal_point_y~%float32 focal_length_x~%float32 focal_length_y~%DistortionCoefficients distortion_coeffs~%================================================================================~%MSG: kortex_driver/DistortionCoefficients~%~%float32 k1~%float32 k2~%float32 k3~%float32 p1~%float32 p2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIntrinsicParametersProfile-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIntrinsicParametersProfile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIntrinsicParametersProfile-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetIntrinsicParametersProfile)))
  'GetIntrinsicParametersProfile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetIntrinsicParametersProfile)))
  'GetIntrinsicParametersProfile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntrinsicParametersProfile)))
  "Returns string type for a service object of type '<GetIntrinsicParametersProfile>"
  "kortex_driver/GetIntrinsicParametersProfile")