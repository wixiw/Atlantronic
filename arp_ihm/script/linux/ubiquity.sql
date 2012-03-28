CREATE DATABASE  `ubiquity` ;

-- phpMyAdmin SQL Dump
-- version 3.3.7deb7
-- http://www.phpmyadmin.net
--
-- Host: localhost
-- Generation Time: Mar 29, 2012 at 12:51 AM
-- Server version: 5.1.49
-- PHP Version: 5.3.3-7+squeeze7

SET SQL_MODE="NO_AUTO_VALUE_ON_ZERO";


/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;

--
-- Database: `ubiquity`
--

-- --------------------------------------------------------

--
-- Table structure for table `orocos_message_to_hmi`
--

CREATE TABLE IF NOT EXISTS `orocos_message_to_hmi` (
  `port_name` varchar(255) NOT NULL,
  `value` varchar(255) NOT NULL,
  UNIQUE KEY `port_name` (`port_name`)
) ENGINE=MyISAM DEFAULT CHARSET=latin1;

--
-- Dumping data for table `orocos_message_to_hmi`
--


-- --------------------------------------------------------

--
-- Table structure for table `ros_message_to_hmi`
--

CREATE TABLE IF NOT EXISTS `ros_message_to_hmi` (
  `topic_name` varchar(255) NOT NULL,
  `value` varchar(255) NOT NULL,
  UNIQUE KEY `topic_name` (`topic_name`)
) ENGINE=MyISAM DEFAULT CHARSET=latin1;

--
-- Dumping data for table `ros_message_to_hmi`
--


CREATE USER 'ard_user'@'%' IDENTIFIED BY  '***';

GRANT SELECT , 
INSERT ,

UPDATE ,
DELETE ,
DROP ,
FILE ON * . * TO  'ard_user'@'%' IDENTIFIED BY  '***' WITH MAX_QUERIES_PER_HOUR 0 MAX_CONNECTIONS_PER_HOUR 0 MAX_UPDATES_PER_HOUR 0 MAX_USER_CONNECTIONS 0 ;

