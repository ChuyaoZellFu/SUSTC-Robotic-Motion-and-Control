; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetIntrinsicParameters-request.msg.html

(cl:defclass <SetIntrinsicParameters-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:IntrinsicParameters
    :initform (cl:make-instance 'kortex_driver-msg:IntrinsicParameters)))
)

(cl:defclass SetIntrinsicParameters-request (<SetIntrinsicParameters-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetIntrinsicParameters-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetIntrinsicParameters-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetIntrinsicParameters-request> is deprecated: use kortex_driver-srv:SetIntrinsicParameters-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetIntrinsicParameters-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetIntrinsicParameters-request>) ostream)
  "Serializes a message object of type '<SetIntrinsicParameters-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetIntrinsicParameters-request>) istream)
  "Deserializes a message object of type '<SetIntrinsicParameters-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetIntrinsicParameters-request>)))
  "Returns string type for a service object of type '<SetIntrinsicParameters-request>"
  "kortex_driver/SetIntrinsicParametersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetIntrinsicParameters-request)))
  "Returns string type for a service object of type 'SetIntrinsicParameters-request"
  "kortex_driver/SetIntrinsicParametersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetIntrinsicParameters-request>)))
  "Returns md5sum for a message object of type '<SetIntrinsicParameters-request>"
  "e4fa2fa37ca4b5105af9a0b157505e8c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetIntrinsicParameters-request)))
  "Returns md5sum for a message object of type 'SetIntrinsicParameters-request"
  "e4fa2fa37ca4b5105af9a0b157505e8c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetIntrinsicParameters-request>)))
  "Returns full string definition for message of type '<SetIntrinsicParameters-request>"
  (cl:format cl:nil "IntrinsicParameters input~%~%================================================================================~%MSG: kortex_driver/IntrinsicParameters~%~%uint32 sensor~%uint32 resolution~%float32 principal_point_x~%float32 principal_point_y~%float32 focal_length_x~%float32 focal_length_y~%DistortionCoefficients distortion_coeffs~%================================================================================~%MSG: kortex_driver/DistortionCoefficients~%~%float32 k1~%float32 k2~%float32 k3~%float32 p1~%float32 p2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetIntrinsicParameters-request)))
  "Returns full string definition for message of type 'SetIntrinsicParameters-request"
  (cl:format cl:nil "IntrinsicParameters input~%~%================================================================================~%MSG: kortex_driver/IntrinsicParameters~%~%uint32 sensor~%uint32 resolution~%float32 principal_point_x~%float32 principal_point_y~%float32 focal_length_x~%float32 focal_length_y~%DistortionCoefficients distortion_coeffs~%================================================================================~%MSG: kortex_driver/DistortionCoefficients~%~%float32 k1~%float32 k2~%float32 k3~%float32 p1~%float32 p2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetIntrinsicParameters-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetIntrinsicParameters-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetIntrinsicParameters-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetIntrinsicParameters-response.msg.html

(cl:defclass <SetIntrinsicParameters-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetIntrinsicParameters-response (<SetIntrinsicParameters-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetIntrinsicParameters-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetIntrinsicParameters-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetIntrinsicParameters-response> is deprecated: use kortex_driver-srv:SetIntrinsicParameters-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetIntrinsicParameters-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetIntrinsicParameters-response>) ostream)
  "Serializes a message object of type '<SetIntrinsicParameters-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetIntrinsicParameters-response>) istream)
  "Deserializes a message object of type '<SetIntrinsicParameters-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetIntrinsicParameters-response>)))
  "Returns string type for a service object of type '<SetIntrinsicParameters-response>"
  "kortex_driver/SetIntrinsicParametersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetIntrinsicParameters-response)))
  "Returns string type for a service object of type 'SetIntrinsicParameters-response"
  "kortex_driver/SetIntrinsicParametersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetIntrinsicParameters-response>)))
  "Returns md5sum for a message object of type '<SetIntrinsicParameters-response>"
  "e4fa2fa37ca4b5105af9a0b157505e8c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetIntrinsicParameters-response)))
  "Returns md5sum for a message object of type 'SetIntrinsicParameters-response"
  "e4fa2fa37ca4b5105af9a0b157505e8c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetIntrinsicParameters-response>)))
  "Returns full string definition for message of type '<SetIntrinsicParameters-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetIntrinsicParameters-response)))
  "Returns full string definition for message of type 'SetIntrinsicParameters-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetIntrinsicParameters-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetIntrinsicParameters-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetIntrinsicParameters-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetIntrinsicParameters)))
  'SetIntrinsicParameters-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetIntrinsicParameters)))
  'SetIntrinsicParameters-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetIntrinsicParameters)))
  "Returns string type for a service object of type '<SetIntrinsicParameters>"
  "kortex_driver/SetIntrinsicParameters")