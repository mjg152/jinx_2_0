/*
 * This file is part of libphidget21
 *
 * Copyright © 2006-2015 Phidgets Inc <patrick@phidgets.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see 
 * <http://www.gnu.org/licenses/>
 */

package com.phidgets.event;

import com.phidgets.Phidget;

/**
 * This class represents the data for a TagGainEvent.
 * 
 * @author Phidgets Inc.
 */
public class TagGainEvent
{
	Phidget source;
	String value;
	int protocol;

	/**
	 * Class constructor. This is called internally by the phidget library when creating this event.
	 * 
	 * @param source the Phidget object from which this event originated
	 */
	public TagGainEvent(Phidget source, String value)
	{
		this.source = source;
		this.value = value;
	}
	public TagGainEvent(Phidget source, String value, int protocol)
	{
		this.source = source;
		this.value = value;
		this.protocol = protocol;
	}

	/**
	 * Returns the source Phidget of this event. This is a reference to the Phidget object from which this
	 * event was called. This object can be cast into a specific type of Phidget object to call specific
	 * device calls on it.
	 * 
	 * @return the event caller
	 */
	public Phidget getSource() {
		return source;
	}

	/**
	 * Returns the gained tag.
	 * 
	 * @return the gained tag
	 */
	public String getValue() {
		return value;
	}

	/**
	 * Returns the protocol of the Tag that was lost.
	 * 
	 * @return the lost tag protocol
	 */
	public int getProtocol() {
		return protocol;
	}

	/**
	 * Returns a string containing information about the event.
	 * 
	 * @return an informative event string
	 */
	public String toString() {
		return source.toString() + " Tag Gained: "
		  + value;
	}
}
