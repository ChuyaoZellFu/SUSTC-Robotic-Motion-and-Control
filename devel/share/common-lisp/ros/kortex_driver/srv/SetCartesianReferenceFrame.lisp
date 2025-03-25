; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetCartesianReferenceFrame-request.msg.html

(cl:defclass <SetCartesianReferenceFrame-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:CartesianReferenceFrameInfo
    :initform (cl:make-instance 'kortex_driver-msg:CartesianReferenceFrameInfo)))
)

(cl:defclass SetCartesianReferenceFrame-request (<SetCartesianReferenceFrame-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCartesianReferenceFrame-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCartesianReferenceFrame-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetCartesianReferenceFrame-request> is deprecated: use kortex_driver-srv:SetCartesianReferenceFrame-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetCartesianReferenceFrame-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCartesianReferenceFrame-request>) ostream)
  "Serializes a message object of type '<SetCartesianReferenceFrame-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCartesianReferenceFrame-request>) istream)
  "Deserializes a message object of type '<SetCartesianReferenceFrame-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCartesianReferenceFrame-request>)))
  "Returns string type for a service object of type '<SetCartesianReferenceFrame-request>"
  "kortex_driver/SetCartesianReferenceFrameRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCartesianReferenceFrame-request)))
  "Returns string type for a service object of type 'SetCartesianReferenceFrame-request"
  "kortex_driver/SetCartesianReferenceFrameRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCartesianReferenceFrame-request>)))
  "Returns md5sum for a message object of type '<SetCartesianReferenceFrame-request>"
  "80417b91d14dfe7b5a3f569e00b74176")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCartesianReferenceFrame-request)))
  "Returns md5sum for a message object of type 'SetCartesianReferenceFrame-request"
  "80417b91d14dfe7b5a3f569e00b74176")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCartesianReferenceFrame-request>)))
  "Returns full string definition for message of type '<SetCartesianReferenceFrame-request>"
  (cl:format cl:nil "CartesianReferenceFrameInfo input~%~%================================================================================~%MSG: kortex_driver/CartesianReferenceFrameInfo~%~%uint32 reference_frame~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCartesianReferenceFrame-request)))
  "Returns full string definition for message of type 'SetCartesianReferenceFrame-request"
  (cl:format cl:nil "CartesianReferenceFrameInfo input~%~%================================================================================~%MSG: kortex_driver/CartesianReferenceFrameInfo~%~%uint32 reference_frame~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCartesianReferenceFrame-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCartesianReferenceFrame-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCartesianReferenceFrame-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetCartesianReferenceFrame-response.msg.html

(cl:defclass <SetCartesianReferenceFrame-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetCartesianReferenceFrame-response (<SetCartesianReferenceFrame-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCartesianReferenceFrame-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCartesianReferenceFrame-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetCartesianReferenceFrame-response> is deprecated: use kortex_driver-srv:SetCartesianReferenceFrame-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetCartesianReferenceFrame-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCartesianReferenceFrame-response>) ostream)
  "Serializes a message object of type '<SetCartesianReferenceFrame-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCartesianReferenceFrame-response>) istream)
  "Deserializes a message object of type '<SetCartesianReferenceFrame-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCartesianReferenceFrame-response>)))
  "Returns string type for a service object of type '<SetCartesianReferenceFrame-response>"
  "kortex_driver/SetCartesianReferenceFrameResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCartesianReferenceFrame-response)))
  "Returns string type for a service object of type 'SetCartesianReferenceFrame-response"
  "kortex_driver/SetCartesianReferenceFrameResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCartesianReferenceFrame-response>)))
  "Returns md5sum for a message object of type '<SetCartesianReferenceFrame-response>"
  "80417b91d14dfe7b5a3f569e00b74176")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCartesianReferenceFrame-response)))
  "Returns md5sum for a message object of type 'SetCartesianReferenceFrame-response"
  "80417b91d14dfe7b5a3f569e00b74176")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCartesianReferenceFrame-response>)))
  "Returns full string definition for message of type '<SetCartesianReferenceFrame-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCartesianReferenceFrame-response)))
  "Returns full string definition for message of type 'SetCartesianReferenceFrame-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCartesianReferenceFrame-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCartesianReferenceFrame-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCartesianReferenceFrame-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCartesianReferenceFrame)))
  'SetCartesianReferenceFrame-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCartesianReferenceFrame)))
  'SetCartesianReferenceFrame-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCartesianReferenceFrame)))
  "Returns string type for a service object of type '<SetCartesianReferenceFrame>"
  "kortex_driver/SetCartesianReferenceFrame")