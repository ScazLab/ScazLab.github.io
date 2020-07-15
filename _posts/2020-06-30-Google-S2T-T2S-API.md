---
layout: post
title: Google Speech2Text and Text2Speech API Reference
author: Debasmita Ghose
description: "Instructions on getting Google's Speech2Text and Text2Speech API to work with ROS"
tags: 
categories: [wiki]
comments: false
permalink: google_s2t_t2s.html
excerpt_separator: <!-- More -->
---

Welcome to the API Reference Guide for Google Speech2Text and Text2Speech. 

<!-- More -->

## Generating GCloud credentials for a new project

NOTE: If you've already created a project and service account, you may skip this step. For wodoto's case, our project is "vector-control", and service account is "scazlab-wodoto". 

Navigate to https://console.cloud.google.com/home, making sure to sign into the Scazlab gmail account. From here, click "select a project" on the top-left. Name your project and leave organization blank. Next, navigate to https://console.cloud.google.com/apis. Make sure to select your newly-created project from the top-left dropdown menu. Select "Library" from the left navigation bar, search for the API you'd like to use, select it and click "ENABLE." This will take some time to load. In wodoto's case, we're using "Cloud Text-to-Speech API" and "Cloud Speech-to-Text API." 

Once you've added all the APIs, again from the left navigation bar select "Credentials." Then, click on "Create Credentials, select "Create service account," give it a name, an optional description, and hit "CREATE." On the next page, give it the role "Owner," and then keep hitting "NEXT" until the service account has been created. 

## Setting up the GCloud key.
Make sure to have your project selected from the top-left menu, and to be logged into the Scazlab gmail account. Find your service account on the "Credentials" tab at https://console.cloud.google.com/apis, under "Service Accounts."  Select it, scroll down, and click on "ADD KEY," then "Create new key", and select the type to be JSON. Your browser should then prompt you to download a .json file. 

Copy the downloaded file in into your repository, or anywhere else on your system, and set the environment variable GOOGLE_APPLICATION_CREDENTIALS by running `export GOOGLE_APPLICATION_CREDENTIALS=<path-to-json>`. If you don't want to set this every time you open bash, consider:
- Adding the environment variable to your .bashrc so that it's automatically set every session by running `echo "export GOOGLE_APPLICATION_CREDENTIALS=<path-to-json>" >> ~/.bashrc`
- Add it to your ROS launch file with the tag `<env name="GOOGLE_APPLICATION_CREDENTIALS" value="<path-to-json>" />`
- Add it to your Dockerfile with the line `ENV GOOGLE_APPLICATION_CREDENTIALS=<path-to-json>`

In either case, congratulations! You should now be able to interact with Google's APIs. 






To edit this document, make changes [here](https://github.com/ScazLab/ScazLab.github.io/blob/master/_posts/2020-06-30-Google-S2T-T2S-API.md)

