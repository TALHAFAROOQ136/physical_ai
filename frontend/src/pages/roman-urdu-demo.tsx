import React from 'react';
import Layout from '@theme/Layout';
import RomanUrduToggle from '../components/RomanUrduToggle';

function RomanUrduDemo() {
  return (
    <Layout title="Roman Urdu Demo" description="Demonstration of Roman Urdu functionality">
      <div className="container margin-vert--lg padding-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>Roman Urdu Functionality Demo</h1>

            <h2>Using Roman Urdu Toggle Component</h2>
            <p>This demonstrates the Roman Urdu toggle functionality:</p>

            <RomanUrduToggle>
              یہ ایک ٹیسٹ پیغام ہے
            </RomanUrduToggle>

            <div className="margin-vert--lg">
              <h2>About Roman Urdu</h2>
              <p>Roman Urdu is the representation of the Urdu language using the Latin script.</p>
              <p>Example: "Hello" in Roman Urdu is "Hello" but "کیا حال ہے" becomes "kya haal hai".</p>
            </div>

            <div className="margin-vert--lg">
              <h2>Language Options</h2>
              <p>With the addition of Roman Urdu, the platform now supports:</p>
              <ul>
                <li>English (en)</li>
                <li>Urdu in Arabic script (ur)</li>
                <li>Roman Urdu in Latin script (ur-r)</li>
              </ul>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default RomanUrduDemo;