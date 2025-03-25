; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetExtrinsicParameters-request.msg.html

(cl:defclass <GetExtrinsicParameters-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetExtrinsicParameters-request (<GetExtrinsicParameters-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetExtrinsicParameters-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetExtrinsicParameters-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetExtrinsicParameters-request> is deprecated: use kortex_driver-srv:GetExtrinsicParameters-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetExtrinsicParameters-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetExtrinsicParameters-request>) ostream)
  "Serializes a message object of type '<GetExtrinsicParameters-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetExtrinsicParameters-request>) istream)
  "Deserializes a message object of type '<GetExtrinsicParameters-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetExtrinsicParameters-request>)))
  "Returns string type for a service object of type '<GetExtrinsicParameters-request>"
  "kortex_driver/GetExtrinsicParametersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetExtrinsicParameters-request)))
  "Returns string type for a service object of type 'GetExtrinsicParameters-request"
  "kortex_driver/GetExtrinsicParametersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetExtrinsicParameters-request>)))
  "Returns md5sum for a message object of type '<GetExtrinsicParameters-request>"
  "769b6e0a4e2cbed0ce69ce84dc1b50e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetExtrinsicParameters-request)))
  "Returns md5sum for a message object of type 'GetExtrinsicParameters-request"
  "769b6e0a4e2cbed0ce69ce84dc1b50e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetExtrinsicParameters-request>)))
  "Returns full string definition for message of type '<GetExtrinsicParameters-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetExtrinsicParameters-request)))
  "Returns full string definition for message of type 'GetExtrinsicParameters-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetExtrinsicParameters-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetExtrinsicParameters-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetExtrinsicParameters-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetExtrinsicParameters-response.msg.html

(cl:defclass <GetExtrinsicParameters-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ExtrinsicParameters
    :initform (cl:make-instance 'kortex_driver-msg:ExtrinsicParameters)))
)

(cl:defclass GetExtrinsicParameters-response (<GetExtrinsicParameters-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetExtrinsicParameters-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetExtrinsicParameters-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetExtrinsicParameters-response> is deprecated: use kortex_driver-srv:GetExtrinsicParameters-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetExtrinsicParameters-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetExtrinsicParameters-response>) ostream)
  "Serializes a message object of type '<GetExtrinsicParameters-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetExtrinsicParameters-response>) istream)
  "Deserializes a message object of type '<GetExtrinsicParameters-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetExtrinsicParameters-response>)))
  "Returns string type for a service object of type '<GetExtrinsicParameters-response>"
  "kortex_driver/GetExtrinsicParametersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetExtrinsicParameters-response)))
  "Returns string type for a service object of type 'GetExtrinsicParameters-response"
  "kortex_driver/GetExtrinsicParametersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetExtrinsicParameters-response>)))
  "Returns md5sum for a message object of type '<GetExtrinsicParameters-response>"
  "769b6e0a4e2cbed0ce69ce84dc1b50e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetExtrinsicParameters-response)))
  "Returns md5sum for a message object of type 'GetExtrinsicParameters-response"
  "769b6e0a4e2cbed0ce69ce84dc1b50e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetExtrinsicParameters-response>)))
  "Returns full string definition for message of type '<GetExtrinsicParameters-response>"
  (cl:format cl:nil "ExtrinsicParameters output~%~%================================================================================~%MSG: kortex_driver/ExtrinsicParameters~%~%VisionConfig_RotationMatrix rotation~%TranslationVector translation~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrix~%~%VisionConfig_RotationMatrixRow row1~%VisionConfig_RotationMatrixRow row2~%VisionConfig_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/TranslationVector~%~%float32 t_x~%float32 t_y~%float32 t_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetExtrinsicParameters-response)))
  "Returns full string definition for message of type 'GetExtrinsicParameters-response"
  (cl:format cl:nil "ExtrinsicParameters output~%~%================================================================================~%MSG: kortex_driver/ExtrinsicParameters~%~%VisionConfig_RotationMatrix rotation~%TranslationVector translation~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrix~%~%VisionConfig_RotationMatrixRow row1~%VisionConfig_RotationMatrixRow row2~%VisionConfig_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/TranslationVector~%~%float32 t_x~%float32 t_y~%float32 t_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetExtrinsicParameters-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetExtrinsicParameters-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetExtrinsicParameters-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetExtrinsicParameters)))
  'GetExtrinsicParameters-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetExtrinsicParameters)))
  'GetExtrinsicParameters-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetExtrinsicParameters)))
  "Returns string type for a service object of type '<GetExtrinsicParameters>"
  "kortex_driver/GetExtrinsicParameters")