**io_manifests** is a collection of manifest files (JSON files)
of rapyuta.io packages that are created in
[sample walkthroughs](https://userdocs.rapyuta.io/build-solutions/sample-walkthroughs/).

Every walkthrough requires the developer to add either ROS or non-ROS
packages using rapyuta.io platform. Alternatively, the packages are
imported by uploading manifests to rapyuta.io platform.

Clone or download the repository, **io_tutorials**, locally.

```
git clone https://github.com/<git-username>/io_tutorials.git
```

where *username* is the developer's Git username.

Now, open [rapyuta.io](https://console.rapyuta.io) platform in a web
browser.

All of the manifests of packages used in the sample walkthroughs
are found in the subdirectory, **io_manifests**

On the left navigation bar, Click **CATALOG** > **IMPORT PACKAGE**.

Upload the required manifest JSON file.

For example, to add the ***MinIO File Server*** package to rapyuta.io
platform, upload its manifest file: **minio_file_server_manifest.json**

[JSONLint](https://jsonlint.com/) is the tool used for formatting all of
the manifests.