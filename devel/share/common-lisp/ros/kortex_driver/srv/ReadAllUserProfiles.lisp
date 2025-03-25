; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ReadAllUserProfiles-request.msg.html

(cl:defclass <ReadAllUserProfiles-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ReadAllUserProfiles-request (<ReadAllUserProfiles-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadAllUserProfiles-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadAllUserProfiles-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadAllUserProfiles-request> is deprecated: use kortex_driver-srv:ReadAllUserProfiles-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ReadAllUserProfiles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadAllUserProfiles-request>) ostream)
  "Serializes a message object of type '<ReadAllUserProfiles-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadAllUserProfiles-request>) istream)
  "Deserializes a message object of type '<ReadAllUserProfiles-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadAllUserProfiles-request>)))
  "Returns string type for a service object of type '<ReadAllUserProfiles-request>"
  "kortex_driver/ReadAllUserProfilesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllUserProfiles-request)))
  "Returns string type for a service object of type 'ReadAllUserProfiles-request"
  "kortex_driver/ReadAllUserProfilesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadAllUserProfiles-request>)))
  "Returns md5sum for a message object of type '<ReadAllUserProfiles-request>"
  "682c67f2598d4e520be0fc6015a67bc4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadAllUserProfiles-request)))
  "Returns md5sum for a message object of type 'ReadAllUserProfiles-request"
  "682c67f2598d4e520be0fc6015a67bc4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadAllUserProfiles-request>)))
  "Returns full string definition for message of type '<ReadAllUserProfiles-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadAllUserProfiles-request)))
  "Returns full string definition for message of type 'ReadAllUserProfiles-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadAllUserProfiles-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadAllUserProfiles-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadAllUserProfiles-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ReadAllUserProfiles-response.msg.html

(cl:defclass <ReadAllUserProfiles-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:UserProfileList
    :initform (cl:make-instance 'kortex_driver-msg:UserProfileList)))
)

(cl:defclass ReadAllUserProfiles-response (<ReadAllUserProfiles-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadAllUserProfiles-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadAllUserProfiles-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadAllUserProfiles-response> is deprecated: use kortex_driver-srv:ReadAllUserProfiles-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ReadAllUserProfiles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadAllUserProfiles-response>) ostream)
  "Serializes a message object of type '<ReadAllUserProfiles-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadAllUserProfiles-response>) istream)
  "Deserializes a message object of type '<ReadAllUserProfiles-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadAllUserProfiles-response>)))
  "Returns string type for a service object of type '<ReadAllUserProfiles-response>"
  "kortex_driver/ReadAllUserProfilesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllUserProfiles-response)))
  "Returns string type for a service object of type 'ReadAllUserProfiles-response"
  "kortex_driver/ReadAllUserProfilesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadAllUserProfiles-response>)))
  "Returns md5sum for a message object of type '<ReadAllUserProfiles-response>"
  "682c67f2598d4e520be0fc6015a67bc4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadAllUserProfiles-response)))
  "Returns md5sum for a message object of type 'ReadAllUserProfiles-response"
  "682c67f2598d4e520be0fc6015a67bc4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadAllUserProfiles-response>)))
  "Returns full string definition for message of type '<ReadAllUserProfiles-response>"
  (cl:format cl:nil "UserProfileList output~%~%================================================================================~%MSG: kortex_driver/UserProfileList~%~%UserProfile[] user_profiles~%================================================================================~%MSG: kortex_driver/UserProfile~%~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadAllUserProfiles-response)))
  "Returns full string definition for message of type 'ReadAllUserProfiles-response"
  (cl:format cl:nil "UserProfileList output~%~%================================================================================~%MSG: kortex_driver/UserProfileList~%~%UserProfile[] user_profiles~%================================================================================~%MSG: kortex_driver/UserProfile~%~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadAllUserProfiles-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadAllUserProfiles-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadAllUserProfiles-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReadAllUserProfiles)))
  'ReadAllUserProfiles-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReadAllUserProfiles)))
  'ReadAllUserProfiles-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllUserProfiles)))
  "Returns string type for a service object of type '<ReadAllUserProfiles>"
  "kortex_driver/ReadAllUserProfiles")