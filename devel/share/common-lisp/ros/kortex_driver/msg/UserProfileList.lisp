; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude UserProfileList.msg.html

(cl:defclass <UserProfileList> (roslisp-msg-protocol:ros-message)
  ((user_profiles
    :reader user_profiles
    :initarg :user_profiles
    :type (cl:vector kortex_driver-msg:UserProfile)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:UserProfile :initial-element (cl:make-instance 'kortex_driver-msg:UserProfile))))
)

(cl:defclass UserProfileList (<UserProfileList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UserProfileList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UserProfileList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<UserProfileList> is deprecated: use kortex_driver-msg:UserProfileList instead.")))

(cl:ensure-generic-function 'user_profiles-val :lambda-list '(m))
(cl:defmethod user_profiles-val ((m <UserProfileList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_profiles-val is deprecated.  Use kortex_driver-msg:user_profiles instead.")
  (user_profiles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UserProfileList>) ostream)
  "Serializes a message object of type '<UserProfileList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'user_profiles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'user_profiles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UserProfileList>) istream)
  "Deserializes a message object of type '<UserProfileList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'user_profiles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'user_profiles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:UserProfile))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UserProfileList>)))
  "Returns string type for a message object of type '<UserProfileList>"
  "kortex_driver/UserProfileList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UserProfileList)))
  "Returns string type for a message object of type 'UserProfileList"
  "kortex_driver/UserProfileList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UserProfileList>)))
  "Returns md5sum for a message object of type '<UserProfileList>"
  "2d71d5ab953fb8ddd6c9d9d4a1379bb4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UserProfileList)))
  "Returns md5sum for a message object of type 'UserProfileList"
  "2d71d5ab953fb8ddd6c9d9d4a1379bb4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UserProfileList>)))
  "Returns full string definition for message of type '<UserProfileList>"
  (cl:format cl:nil "~%UserProfile[] user_profiles~%================================================================================~%MSG: kortex_driver/UserProfile~%~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UserProfileList)))
  "Returns full string definition for message of type 'UserProfileList"
  (cl:format cl:nil "~%UserProfile[] user_profiles~%================================================================================~%MSG: kortex_driver/UserProfile~%~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UserProfileList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'user_profiles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UserProfileList>))
  "Converts a ROS message object to a list"
  (cl:list 'UserProfileList
    (cl:cons ':user_profiles (user_profiles msg))
))
