; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude FirmwareBundleVersions.msg.html

(cl:defclass <FirmwareBundleVersions> (roslisp-msg-protocol:ros-message)
  ((main_bundle_version
    :reader main_bundle_version
    :initarg :main_bundle_version
    :type cl:string
    :initform "")
   (components_versions
    :reader components_versions
    :initarg :components_versions
    :type (cl:vector kortex_driver-msg:FirmwareComponentVersion)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:FirmwareComponentVersion :initial-element (cl:make-instance 'kortex_driver-msg:FirmwareComponentVersion))))
)

(cl:defclass FirmwareBundleVersions (<FirmwareBundleVersions>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FirmwareBundleVersions>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FirmwareBundleVersions)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<FirmwareBundleVersions> is deprecated: use kortex_driver-msg:FirmwareBundleVersions instead.")))

(cl:ensure-generic-function 'main_bundle_version-val :lambda-list '(m))
(cl:defmethod main_bundle_version-val ((m <FirmwareBundleVersions>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:main_bundle_version-val is deprecated.  Use kortex_driver-msg:main_bundle_version instead.")
  (main_bundle_version m))

(cl:ensure-generic-function 'components_versions-val :lambda-list '(m))
(cl:defmethod components_versions-val ((m <FirmwareBundleVersions>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:components_versions-val is deprecated.  Use kortex_driver-msg:components_versions instead.")
  (components_versions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FirmwareBundleVersions>) ostream)
  "Serializes a message object of type '<FirmwareBundleVersions>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'main_bundle_version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'main_bundle_version))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'components_versions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'components_versions))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FirmwareBundleVersions>) istream)
  "Deserializes a message object of type '<FirmwareBundleVersions>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'main_bundle_version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'main_bundle_version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'components_versions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'components_versions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:FirmwareComponentVersion))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FirmwareBundleVersions>)))
  "Returns string type for a message object of type '<FirmwareBundleVersions>"
  "kortex_driver/FirmwareBundleVersions")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FirmwareBundleVersions)))
  "Returns string type for a message object of type 'FirmwareBundleVersions"
  "kortex_driver/FirmwareBundleVersions")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FirmwareBundleVersions>)))
  "Returns md5sum for a message object of type '<FirmwareBundleVersions>"
  "c4f9de789ae8af0bd7b668a4f8f7ed49")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FirmwareBundleVersions)))
  "Returns md5sum for a message object of type 'FirmwareBundleVersions"
  "c4f9de789ae8af0bd7b668a4f8f7ed49")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FirmwareBundleVersions>)))
  "Returns full string definition for message of type '<FirmwareBundleVersions>"
  (cl:format cl:nil "~%string main_bundle_version~%FirmwareComponentVersion[] components_versions~%================================================================================~%MSG: kortex_driver/FirmwareComponentVersion~%~%string name~%string version~%uint32 device_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FirmwareBundleVersions)))
  "Returns full string definition for message of type 'FirmwareBundleVersions"
  (cl:format cl:nil "~%string main_bundle_version~%FirmwareComponentVersion[] components_versions~%================================================================================~%MSG: kortex_driver/FirmwareComponentVersion~%~%string name~%string version~%uint32 device_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FirmwareBundleVersions>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'main_bundle_version))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'components_versions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FirmwareBundleVersions>))
  "Converts a ROS message object to a list"
  (cl:list 'FirmwareBundleVersions
    (cl:cons ':main_bundle_version (main_bundle_version msg))
    (cl:cons ':components_versions (components_versions msg))
))
