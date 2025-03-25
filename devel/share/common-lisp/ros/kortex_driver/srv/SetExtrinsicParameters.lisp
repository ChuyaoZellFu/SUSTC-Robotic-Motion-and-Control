; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetExtrinsicParameters-request.msg.html

(cl:defclass <SetExtrinsicParameters-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ExtrinsicParameters
    :initform (cl:make-instance 'kortex_driver-msg:ExtrinsicParameters)))
)

(cl:defclass SetExtrinsicParameters-request (<SetExtrinsicParameters-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetExtrinsicParameters-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetExtrinsicParameters-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetExtrinsicParameters-request> is deprecated: use kortex_driver-srv:SetExtrinsicParameters-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetExtrinsicParameters-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetExtrinsicParameters-request>) ostream)
  "Serializes a message object of type '<SetExtrinsicParameters-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetExtrinsicParameters-request>) istream)
  "Deserializes a message object of type '<SetExtrinsicParameters-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetExtrinsicParameters-request>)))
  "Returns string type for a service object of type '<SetExtrinsicParameters-request>"
  "kortex_driver/SetExtrinsicParametersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetExtrinsicParameters-request)))
  "Returns string type for a service object of type 'SetExtrinsicParameters-request"
  "kortex_driver/SetExtrinsicParametersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetExtrinsicParameters-request>)))
  "Returns md5sum for a message object of type '<SetExtrinsicParameters-request>"
  "4b2d31a314d526e9790309739cc1aeec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetExtrinsicParameters-request)))
  "Returns md5sum for a message object of type 'SetExtrinsicParameters-request"
  "4b2d31a314d526e9790309739cc1aeec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetExtrinsicParameters-request>)))
  "Returns full string definition for message of type '<SetExtrinsicParameters-request>"
  (cl:format cl:nil "ExtrinsicParameters input~%~%================================================================================~%MSG: kortex_driver/ExtrinsicParameters~%~%VisionConfig_RotationMatrix rotation~%TranslationVector translation~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrix~%~%VisionConfig_RotationMatrixRow row1~%VisionConfig_RotationMatrixRow row2~%VisionConfig_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/TranslationVector~%~%float32 t_x~%float32 t_y~%float32 t_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetExtrinsicParameters-request)))
  "Returns full string definition for message of type 'SetExtrinsicParameters-request"
  (cl:format cl:nil "ExtrinsicParameters input~%~%================================================================================~%MSG: kortex_driver/ExtrinsicParameters~%~%VisionConfig_RotationMatrix rotation~%TranslationVector translation~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrix~%~%VisionConfig_RotationMatrixRow row1~%VisionConfig_RotationMatrixRow row2~%VisionConfig_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/TranslationVector~%~%float32 t_x~%float32 t_y~%float32 t_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetExtrinsicParameters-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetExtrinsicParameters-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetExtrinsicParameters-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetExtrinsicParameters-response.msg.html

(cl:defclass <SetExtrinsicParameters-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetExtrinsicParameters-response (<SetExtrinsicParameters-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetExtrinsicParameters-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetExtrinsicParameters-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetExtrinsicParameters-response> is deprecated: use kortex_driver-srv:SetExtrinsicParameters-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetExtrinsicParameters-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetExtrinsicParameters-response>) ostream)
  "Serializes a message object of type '<SetExtrinsicParameters-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetExtrinsicParameters-response>) istream)
  "Deserializes a message object of type '<SetExtrinsicParameters-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetExtrinsicParameters-response>)))
  "Returns string type for a service object of type '<SetExtrinsicParameters-response>"
  "kortex_driver/SetExtrinsicParametersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetExtrinsicParameters-response)))
  "Returns string type for a service object of type 'SetExtrinsicParameters-response"
  "kortex_driver/SetExtrinsicParametersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetExtrinsicParameters-response>)))
  "Returns md5sum for a message object of type '<SetExtrinsicParameters-response>"
  "4b2d31a314d526e9790309739cc1aeec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetExtrinsicParameters-response)))
  "Returns md5sum for a message object of type 'SetExtrinsicParameters-response"
  "4b2d31a314d526e9790309739cc1aeec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetExtrinsicParameters-response>)))
  "Returns full string definition for message of type '<SetExtrinsicParameters-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetExtrinsicParameters-response)))
  "Returns full string definition for message of type 'SetExtrinsicParameters-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetExtrinsicParameters-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetExtrinsicParameters-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetExtrinsicParameters-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetExtrinsicParameters)))
  'SetExtrinsicParameters-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetExtrinsicParameters)))
  'SetExtrinsicParameters-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetExtrinsicParameters)))
  "Returns string type for a service object of type '<SetExtrinsicParameters>"
  "kortex_driver/SetExtrinsicParameters")