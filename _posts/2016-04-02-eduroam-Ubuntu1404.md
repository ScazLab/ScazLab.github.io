---
layout: post
title: How to connect to eduroam on Ubuntu 14.04
author: Alessandro Roncone
description: ""
tags: [tutorial,ubuntu]
categories: [wiki]
comments: false
excerpt_separator: <!-- More -->
---

Here are the settings to help you connect to `eduroam` on Ubuntu 14.04:

<!-- More -->

 * _Security_: **WPA & WPA2 Enterprise**
 * _Authentication_: **Protected EAP (PEAP)**
 * _Anonymous identity_: **anonymous@yale.edu**
 * _CA Certificate_: **navigate to `/etc/ssl/certs/` and select `CA_USTORE_eduroam.pem`**
 * _PEAP version_: **Automatic**
 * _Inner authentication_: **MSCHAPv2**
 * _Username_: `your_netID@yale.edu`
 * _Password_: `your_netID_password`

