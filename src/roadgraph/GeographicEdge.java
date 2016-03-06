package roadgraph;

import geography.GeographicPoint;

/**
 * An Edge that connects two points
 * Edge has 3 properties name, type,length
 * Created by ksama on 3/6/16.
 */
public class GeographicEdge {
  private GeographicPoint from;
  private GeographicPoint to;

  private String edgeName;
  private String edgeType;

  public GeographicPoint getTo() {
    return to;
  }

  // Length in km
  private double length;

  public GeographicEdge(GeographicPoint from, GeographicPoint to, String edgeName,
                        String edgeType, double length)
  {
    this.from = from;
    this.to = to;
    this.edgeName = edgeName;
    this.edgeType = edgeType;
    this.length = length;
  }


  /** Two edges are equal if they have the same start and end points
   *  and they have the same edge name.
   */
  public boolean equals(Object o)
  {
    if (!(o instanceof GeographicEdge)) {
      return false;
    }

    GeographicEdge other = (GeographicEdge)o;
    boolean ptsEqual = false;
    if (other.from.equals(this.from) && other.to.equals(this.to)) {
      ptsEqual = true;
    }
    if (other.to.equals(this.from) && other.from.equals(this.to))
    {
      ptsEqual = true;
    }
    return this.edgeName.equals(other.edgeName) && ptsEqual && this.length == other.length;
  }

  // get hashCode
  public int hashCode()
  {
    return from.hashCode() + to.hashCode();
  }

  // return road segment as String
  public String toString()
  {
    String toReturn = this.edgeName + ", " +this.edgeType;
    toReturn += " [" + from;
    toReturn += "; " + to + "]";

    return toReturn;
  }

  // get the length of the road segment
  public double getLength() { return this.length; }



}
