; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetFirmwareBundleVersions-request.msg.html

(cl:defclass <GetFirmwareBundleVersions-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetFirmwareBundleVersions-request (<GetFirmwareBundleVersions-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetFirmwareBundleVersions-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetFirmwareBundleVersions-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetFirmwareBundleVersions-request> is deprecated: use kortex_driver-srv:GetFirmwareBundleVersions-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetFirmwareBundleVersions-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetFirmwareBundleVersions-request>) ostream)
  "Serializes a message object of type '<GetFirmwareBundleVersions-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetFirmwareBundleVersions-request>) istream)
  "Deserializes a message object of type '<GetFirmwareBundleVersions-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetFirmwareBundleVersions-request>)))
  "Returns string type for a service object of type '<GetFirmwareBundleVersions-request>"
  "kortex_driver/GetFirmwareBundleVersionsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFirmwareBundleVersions-request)))
  "Returns string type for a service object of type 'GetFirmwareBundleVersions-request"
  "kortex_driver/GetFirmwareBundleVersionsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetFirmwareBundleVersions-request>)))
  "Returns md5sum for a message object of type '<GetFirmwareBundleVersions-request>"
  "8a198c1bd1965cc2d1e06f5a3548e004")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetFirmwareBundleVersions-request)))
  "Returns md5sum for a message object of type 'GetFirmwareBundleVersions-request"
  "8a198c1bd1965cc2d1e06f5a3548e004")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetFirmwareBundleVersions-request>)))
  "Returns full string definition for message of type '<GetFirmwareBundleVersions-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetFirmwareBundleVersions-request)))
  "Returns full string definition for message of type 'GetFirmwareBundleVersions-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetFirmwareBundleVersions-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetFirmwareBundleVersions-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetFirmwareBundleVersions-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetFirmwareBundleVersions-response.msg.html

(cl:defclass <GetFirmwareBundleVersions-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:FirmwareBundleVersions
    :initform (cl:make-instance 'kortex_driver-msg:FirmwareBundleVersions)))
)

(cl:defclass GetFirmwareBundleVersions-response (<GetFirmwareBundleVersions-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetFirmwareBundleVersions-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetFirmwareBundleVersions-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetFirmwareBundleVersions-response> is deprecated: use kortex_driver-srv:GetFirmwareBundleVersions-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetFirmwareBundleVersions-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetFirmwareBundleVersions-response>) ostream)
  "Serializes a message object of type '<GetFirmwareBundleVersions-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetFirmwareBundleVersions-response>) istream)
  "Deserializes a message object of type '<GetFirmwareBundleVersions-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetFirmwareBundleVersions-response>)))
  "Returns string type for a service object of type '<GetFirmwareBundleVersions-response>"
  "kortex_driver/GetFirmwareBundleVersionsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFirmwareBundleVersions-response)))
  "Returns string type for a service object of type 'GetFirmwareBundleVersions-response"
  "kortex_driver/GetFirmwareBundleVersionsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetFirmwareBundleVersions-response>)))
  "Returns md5sum for a message object of type '<GetFirmwareBundleVersions-response>"
  "8a198c1bd1965cc2d1e06f5a3548e004")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetFirmwareBundleVersions-response)))
  "Returns md5sum for a message object of type 'GetFirmwareBundleVersions-response"
  "8a198c1bd1965cc2d1e06f5a3548e004")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetFirmwareBundleVersions-response>)))
  "Returns full string definition for message of type '<GetFirmwareBundleVersions-response>"
  (cl:format cl:nil "FirmwareBundleVersions output~%~%================================================================================~%MSG: kortex_driver/FirmwareBundleVersions~%~%string main_bundle_version~%FirmwareComponentVersion[] components_versions~%================================================================================~%MSG: kortex_driver/FirmwareComponentVersion~%~%string name~%string version~%uint32 device_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetFirmwareBundleVersions-response)))
  "Returns full string definition for message of type 'GetFirmwareBundleVersions-response"
  (cl:format cl:nil "FirmwareBundleVersions output~%~%================================================================================~%MSG: kortex_driver/FirmwareBundleVersions~%~%string main_bundle_version~%FirmwareComponentVersion[] components_versions~%================================================================================~%MSG: kortex_driver/FirmwareComponentVersion~%~%string name~%string version~%uint32 device_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetFirmwareBundleVersions-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetFirmwareBundleVersions-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetFirmwareBundleVersions-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetFirmwareBundleVersions)))
  'GetFirmwareBundleVersions-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetFirmwareBundleVersions)))
  'GetFirmwareBundleVersions-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFirmwareBundleVersions)))
  "Returns string type for a service object of type '<GetFirmwareBundleVersions>"
  "kortex_driver/GetFirmwareBundleVersions")